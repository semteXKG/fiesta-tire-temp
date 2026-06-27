#include "tire_segment.h"
#include "mlx90640.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIRE_THRESHOLD_OFFSET   5.0f
#define TIRE_MIN_PIXELS         30

/* Set to 1 to detect cold test targets (e.g., coolbag). Default 0 detects hot tires. */
#ifndef TIRE_DETECT_COLD
#define TIRE_DETECT_COLD        0
#endif

typedef struct {
    float proj;
    float temp;
} proj_t;

/*
 * Work buffers are allocated statically because the largest ones exceed the
 * stack budget of the calling FreeRTOS task.  The segmenter is called from a
 * single task, so non-reentrancy is acceptable, and no dynamic allocation is
 * used anywhere in this module.
 */
static float sorted[MLX90640_PIXELS];
static bool  mask[MLX90640_PIXELS];
static int16_t labels[MLX90640_PIXELS];
static proj_t projs[MLX90640_PIXELS];
static uint16_t queue[MLX90640_PIXELS];

static int compare_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static int compare_proj(const void *a, const void *b)
{
    float pa = ((const proj_t *)a)->proj;
    float pb = ((const proj_t *)b)->proj;
    return (pa > pb) - (pa < pb);
}

static float percentile25(float *buf, size_t n)
{
    qsort(buf, n, sizeof(float), compare_float);
    size_t idx = n / 4;
    if (idx >= n) {
        idx = n - 1;
    }
    return buf[idx];
}

/*
 * 4-connected flood fill using a simple queue.  Each pixel is enqueued at most
 * once, so MLX90640_PIXELS entries are sufficient.
 */
static void label_component(int w, int h, int sx, int sy, int16_t label,
                            int *cx, int *cy, int *count)
{
    int qh = 0, qt = 0;
    long sx_sum = 0, sy_sum = 0, cnt = 0;

    int start = sy * w + sx;
    queue[qt++] = (uint16_t)start;
    labels[start] = label;

    while (qh < qt) {
        int idx = queue[qh++];
        int x = idx % w;
        int y = idx / w;

        sx_sum += x;
        sy_sum += y;
        cnt++;

        /* right */
        if (x + 1 < w && mask[idx + 1] && labels[idx + 1] == 0) {
            labels[idx + 1] = label;
            queue[qt++] = (uint16_t)(idx + 1);
        }
        /* left */
        if (x - 1 >= 0 && mask[idx - 1] && labels[idx - 1] == 0) {
            labels[idx - 1] = label;
            queue[qt++] = (uint16_t)(idx - 1);
        }
        /* down */
        if (y + 1 < h && mask[idx + w] && labels[idx + w] == 0) {
            labels[idx + w] = label;
            queue[qt++] = (uint16_t)(idx + w);
        }
        /* up */
        if (y - 1 >= 0 && mask[idx - w] && labels[idx - w] == 0) {
            labels[idx - w] = label;
            queue[qt++] = (uint16_t)(idx - w);
        }
    }

    *cx = (cnt > 0) ? (int)(sx_sum / cnt) : 0;
    *cy = (cnt > 0) ? (int)(sy_sum / cnt) : 0;
    *count = (int)cnt;
}

esp_err_t tire_segment_process(const float *temps, float ta, tire_segment_result_t *out)
{
    if (temps == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->ta = ta;

    memcpy(sorted, temps, sizeof(sorted));
    float bg = percentile25(sorted, MLX90640_PIXELS);
    memset(mask, 0, sizeof(mask));
    memset(labels, 0, sizeof(labels));

    for (int i = 0; i < MLX90640_PIXELS; i++) {
#if TIRE_DETECT_COLD
        mask[i] = temps[i] < (bg - TIRE_THRESHOLD_OFFSET);
#else
        mask[i] = temps[i] > (bg + TIRE_THRESHOLD_OFFSET);
#endif
    }

    int best_count = 0;
    int best_cx = 0, best_cy = 0;
    int16_t best_label = 0;
    int16_t label = 0;

    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (mask[idx] && labels[idx] == 0) {
                int cx, cy, cnt;
                label_component(MLX90640_COLS, MLX90640_ROWS, x, y, ++label,
                                &cx, &cy, &cnt);
                if (cnt > best_count) {
                    best_count = cnt;
                    best_cx = cx;
                    best_cy = cy;
                    best_label = label;
                }
            }
        }
    }

    if (best_count < TIRE_MIN_PIXELS) {
        out->detected = false;
        out->pixels = 0;
        return ESP_OK;
    }

    /* Compute second moments around the largest component's centroid. */
    float mxx = 0.0f, myy = 0.0f, mxy = 0.0f;
    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (labels[idx] == best_label) {
                float dx = (float)x - (float)best_cx;
                float dy = (float)y - (float)best_cy;
                mxx += dx * dx;
                myy += dy * dy;
                mxy += dx * dy;
            }
        }
    }

    float theta = 0.5f * atan2f(2.0f * mxy, mxx - myy);
    float ux = cosf(theta);
    float uy = sinf(theta);

    /* Project tire pixels onto the principal axis and collect temperatures. */
    int n = 0;
    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (labels[idx] == best_label) {
                float dx = (float)x - (float)best_cx;
                float dy = (float)y - (float)best_cy;
                projs[n].proj = dx * ux + dy * uy;
                projs[n].temp = temps[idx];
                n++;
            }
        }
    }

    qsort(projs, n, sizeof(proj_t), compare_proj);

    int n1 = n / 3;
    int n2 = 2 * n / 3;

    float sum_out = 0.0f, sum_cen = 0.0f, sum_in = 0.0f;
    for (int i = 0; i < n1; i++) {
        sum_out += projs[i].temp;
    }
    for (int i = n1; i < n2; i++) {
        sum_cen += projs[i].temp;
    }
    for (int i = n2; i < n; i++) {
        sum_in += projs[i].temp;
    }

    out->detected = true;
    out->pixels = (uint16_t)n;
    out->outside = sum_out / (float)n1;
    out->center = sum_cen / (float)(n2 - n1);
    out->inside = sum_in / (float)(n - n2);

    return ESP_OK;
}

int tire_segment_raw_json(uint32_t timestamp_ms, float ta, const float *pixels, size_t n, char *buf, size_t buflen)
{
    if (pixels == NULL || buf == NULL || buflen == 0) {
        return -1;
    }
    int pos = snprintf(buf, buflen, "{\"ts\":%u,\"ta\":%.1f,\"pixels\":[",
                       (unsigned int)timestamp_ms, ta);
    if (pos < 0 || (size_t)pos >= buflen) {
        return -1;
    }
    for (size_t i = 0; i < n; i++) {
        int written = snprintf(buf + pos, buflen - pos, "%.1f", pixels[i]);
        if (written < 0 || (size_t)(pos + written) >= buflen) {
            return -1;
        }
        pos += written;
        if (i + 1 < n) {
            if ((size_t)pos + 1 >= buflen) {
                return -1;
            }
            buf[pos++] = ',';
        }
    }
    int tail = snprintf(buf + pos, buflen - pos, "]}");
    if (tail < 0 || (size_t)(pos + tail) >= buflen) {
        return -1;
    }
    return pos + tail;
}

int tire_segment_json(const tire_segment_result_t *r, char *buf, size_t buflen)
{
    if (r == NULL || buf == NULL || buflen == 0) {
        return -1;
    }

    if (r->detected) {
        return snprintf(buf, buflen,
                        "{\"ts\":%u,\"ta\":%.1f,\"outside\":%.1f,\"center\":%.1f,\"inside\":%.1f,\"detected\":true,\"pixels\":%u}",
                        (unsigned int)r->timestamp_ms,
                        r->ta, r->outside, r->center, r->inside, r->pixels);
    } else {
        return snprintf(buf, buflen,
                        "{\"ts\":%u,\"ta\":%.1f,\"detected\":false,\"pixels\":%u}",
                        (unsigned int)r->timestamp_ms,
                        r->ta, r->pixels);
    }
}
