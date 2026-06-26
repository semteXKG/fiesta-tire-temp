# Tire temperature segmentation implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `tire_segment` module that extracts outside/center/inside average temperatures from each MLX90640 frame and publishes them as a single JSON payload over MQTT.

**Architecture:** A new `tire_segment.c/h` module receives the 32×24 temperature matrix, thresholds it against the 25th-percentile background, finds the largest connected component, computes its principal axis, splits the component into three equal-count bands along that axis, and averages the temperatures. `tire_temp.c` calls this module once per second and publishes the resulting JSON via `mqttcomm_publish()`.

**Tech Stack:** ESP-IDF v5.5, ESP32, C11, new `driver/i2c_master.h` I²C driver, existing `mqttcomm` publisher.

## Global Constraints

- Target MCU: ESP32 (not ESP32-S3).
- ESP-IDF version: >= 5.5.2.
- Use the new I²C master driver (`driver/i2c_master.h`); do not revert to legacy `driver/i2c.h`.
- `tire_temp` task stack: 8192 bytes.
- MLX90640 frame rate: 1 Hz; segment processing must finish well within 1 s.
- No host unit-test framework exists; verify with `idf.py build` plus hardware-in-the-loop.
- MQTT topic prefix: `fiesta/tire-temp/` (existing client ID).
- Keep files small and focused; segmenter must not know about I²C, MQTT, or FreeRTOS.

## File structure

- **Create** `main/tire_segment.h` — public result type and API.
- **Create** `main/tire_segment.c` — pure image-processing logic.
- **Modify** `main/tire_temp.c` — replace full-matrix logging with segmenter call + JSON publish.
- **Modify** `main/CMakeLists.txt` — add `tire_segment.c` to `SRCS`.

---

### Task 1: Create `main/tire_segment.h`

**Files:**
- Create: `main/tire_segment.h`
- Modify: none
- Test: `idf.py build` (compilation only)

**Interfaces:**
- Consumes: `MLX90640_PIXELS`, `MLX90640_COLS`, `MLX90640_ROWS` from `mlx90640.h`.
- Produces:
  - `typedef struct { bool detected; uint16_t pixels; float outside; float center; float inside; float ta; } tire_segment_result_t;`
  - `esp_err_t tire_segment_process(const float *temps, float ta, tire_segment_result_t *out);`
  - `int tire_segment_json(const tire_segment_result_t *r, char *buf, size_t buflen);`

- [ ] **Step 1: Write the header**

```c
#ifndef TIRE_SEGMENT_H
#define TIRE_SEGMENT_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool detected;
    uint16_t pixels;
    float outside;
    float center;
    float inside;
    float ta;
} tire_segment_result_t;

esp_err_t tire_segment_process(const float *temps, float ta, tire_segment_result_t *out);
int tire_segment_json(const tire_segment_result_t *r, char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* TIRE_SEGMENT_H */
```

- [ ] **Step 2: Verify build**

Run:
```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh && idf.py build
```

Expected: build succeeds (no other file references the new header yet).

- [ ] **Step 3: Commit**

```bash
git add main/tire_segment.h
git commit -m "feat(tire_segment): add segmenter public API header"
```

---

### Task 2: Implement `main/tire_segment.c`

**Files:**
- Create: `main/tire_segment.c`
- Modify: none
- Test: `idf.py build` plus on-device UART dump

**Interfaces:**
- Consumes: 32×24 `float` array and `ta` from `mlx90640_read_frame()`.
- Produces: `tire_segment_result_t` populated with detected flag, pixel count, and three averages; JSON formatter.

- [ ] **Step 1: Implement threshold, connected-component, principal-axis, and split logic**

```c
#include "tire_segment.h"
#include "mlx90640.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define TIRE_THRESHOLD_OFFSET   5.0f
#define TIRE_MIN_PIXELS         30

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
    if (idx >= n) idx = n - 1;
    return buf[idx];
}

static void flood_fill(const bool *mask, bool *visited, int w, int h,
                       int x, int y, int *cx, int *cy, int *count)
{
    int stack[MLX90640_PIXELS][2];
    int sp = 0;
    stack[sp][0] = x;
    stack[sp][1] = y;
    sp++;

    long sx = 0, sy = 0, cnt = 0;

    while (sp > 0) {
        sp--;
        int px = stack[sp][0];
        int py = stack[sp][1];
        int idx = py * w + px;

        if (px < 0 || px >= w || py < 0 || py >= h) continue;
        if (visited[idx] || !mask[idx]) continue;

        visited[idx] = true;
        sx += px;
        sy += py;
        cnt++;

        if (sp + 4 <= MLX90640_PIXELS) {
            stack[sp][0] = px + 1; stack[sp][1] = py; sp++;
            stack[sp][0] = px - 1; stack[sp][1] = py; sp++;
            stack[sp][0] = px;     stack[sp][1] = py + 1; sp++;
            stack[sp][0] = px;     stack[sp][1] = py - 1; sp++;
        }
    }

    *cx = (int)(sx / cnt);
    *cy = (int)(sy / cnt);
    *count = (int)cnt;
}

esp_err_t tire_segment_process(const float *temps, float ta, tire_segment_result_t *out)
{
    if (temps == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->ta = ta;

    float sorted[MLX90640_PIXELS];
    memcpy(sorted, temps, sizeof(sorted));
    float bg = percentile25(sorted, MLX90640_PIXELS);
    float thresh = bg + TIRE_THRESHOLD_OFFSET;

    bool mask[MLX90640_PIXELS];
    bool visited[MLX90640_PIXELS];
    memset(mask, 0, sizeof(mask));
    memset(visited, 0, sizeof(visited));

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        mask[i] = temps[i] > thresh;
    }

    int best_count = 0;
    int best_cx = 0, best_cy = 0;

    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (mask[idx] && !visited[idx]) {
                int cx, cy, cnt;
                flood_fill(mask, visited, MLX90640_COLS, MLX90640_ROWS, x, y, &cx, &cy, &cnt);
                if (cnt > best_count) {
                    best_count = cnt;
                    best_cx = cx;
                    best_cy = cy;
                }
            }
        }
    }

    if (best_count < TIRE_MIN_PIXELS) {
        out->detected = false;
        out->pixels = 0;
        return ESP_OK;
    }

    /* Compute second moments around the largest component's centroid */
    float mxx = 0.0f, myy = 0.0f, mxy = 0.0f;
    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (visited[idx]) {
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

    /* Project tire pixels onto principal axis and copy into a small buffer */
    typedef struct { float proj; float temp; } proj_t;
    proj_t projs[MLX90640_PIXELS];
    int n = 0;
    for (int y = 0; y < MLX90640_ROWS; y++) {
        for (int x = 0; x < MLX90640_COLS; x++) {
            int idx = y * MLX90640_COLS + x;
            if (visited[idx]) {
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
    for (int i = 0; i < n1; i++) sum_out += projs[i].temp;
    for (int i = n1; i < n2; i++) sum_cen += projs[i].temp;
    for (int i = n2; i < n; i++) sum_in += projs[i].temp;

    out->detected = true;
    out->pixels = (uint16_t)n;
    out->outside = sum_out / (float)n1;
    out->center = sum_cen / (float)(n2 - n1);
    out->inside = sum_in / (float)(n - n2);

    return ESP_OK;
}

int tire_segment_json(const tire_segment_result_t *r, char *buf, size_t buflen)
{
    if (r == NULL || buf == NULL || buflen == 0) {
        return -1;
    }

    if (r->detected) {
        return snprintf(buf, buflen,
                        "{\"ts\":%u,\"ta\":%.1f,\"outside\":%.1f,\"center\":%.1f,\"inside\":%.1f,\"detected\":true,\"pixels\":%u}",
                        (unsigned)xTaskGetTickCount() * portTICK_PERIOD_MS,
                        r->ta, r->outside, r->center, r->inside, r->pixels);
    } else {
        return snprintf(buf, buflen,
                        "{\"ts\":%u,\"ta\":%.1f,\"detected\":false,\"pixels\":%u}",
                        (unsigned)xTaskGetTickCount() * portTICK_PERIOD_MS,
                        r->ta, r->pixels);
    }
}
```

- [ ] **Step 2: Add required includes for `snprintf` and `xTaskGetTickCount`**

Already included: `<stdio.h>` for `snprintf`. Add FreeRTOS headers at the top of `tire_segment.c`:

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

- [ ] **Step 3: Verify build**

Run:
```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh && idf.py build
```

Expected: build succeeds (header is present; `CMakeLists.txt` update comes in Task 4).

- [ ] **Step 4: Commit**

```bash
git add main/tire_segment.c
git commit -m "feat(tire_segment): implement threshold, connected-component, and principal-axis split"
```

---

### Task 3: Wire segmenter into `tire_temp.c`

**Files:**
- Create: none
- Modify: `main/tire_temp.c`
- Test: `idf.py build` plus on-device UART/MQTT observation

**Interfaces:**
- Consumes: `tire_segment_process()`, `tire_segment_json()`, `mqttcomm_publish()`.
- Produces: JSON string published once per second to `fiesta/tire-temp/tire-temp`.

- [ ] **Step 1: Add include and reduce matrix logging**

Add near the top of `main/tire_temp.c`:

```c
#include "tire_segment.h"
#include "mqttcomm.h"
```

- [ ] **Step 2: Replace the per-row matrix log with segmenter output**

Inside the inner `while (1)` loop, after the frame-read retry block and before the delay, replace the 24-row `ESP_LOGI` dump with:

```c
            tire_segment_result_t seg;
            err = tire_segment_process(matrix, ta, &seg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "segmentation failed: %s", esp_err_to_name(err));
                break;
            }

            char json[256];
            int len = tire_segment_json(&seg, json, sizeof(json));
            if (len > 0 && len < (int)sizeof(json)) {
                ESP_LOGI(TAG, "%s", json);
                mqttcomm_publish("fiesta/tire-temp/tire-temp", json, len);
            } else {
                ESP_LOGE(TAG, "JSON encoding failed or truncated");
            }
```

Keep the `ESP_LOGI(TAG, "frame %d  Ta=%.1f ...")` summary line for debugging.

- [ ] **Step 3: Verify build**

Run:
```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh && idf.py build
```

Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add main/tire_temp.c
git commit -m "feat(tire_temp): run segmenter and publish JSON per frame"
```

---

### Task 4: Register `tire_segment.c` in `main/CMakeLists.txt`

**Files:**
- Create: none
- Modify: `main/CMakeLists.txt`
- Test: `idf.py build`

**Interfaces:**
- Consumes: existing CMake file.
- Produces: updated source list including `tire_segment.c`.

- [ ] **Step 1: Add `tire_segment.c` to SRCS**

Read `main/CMakeLists.txt` and add `"tire_segment.c"` to the `SRCS` list. Expected result:

```cmake
idf_component_register(SRCS "main.c" "wlan.c" "mqttcomm.c" "tire_temp.c" "mlx90640.c" "tire_segment.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c mqtt nvs_flash)
```

- [ ] **Step 2: Verify build**

Run:
```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh && idf.py build
```

Expected: build succeeds with all objects linked.

- [ ] **Step 3: Commit**

```bash
git add main/CMakeLists.txt
git commit -m "build: add tire_segment.c to CMake sources"
```

---

### Task 5: Hardware-in-the-loop validation

**Files:**
- Create: none
- Modify: none
- Test: on-device with coolbag/warm object

**Interfaces:**
- Consumes: built firmware.
- Produces: verified UART/JSON output.

- [ ] **Step 1: Flash the firmware**

Run:
```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh && idf.py -p /dev/ttyUSB0 flash
```

- [ ] **Step 2: Point sensor at coolbag and capture UART**

Run:
```bash
idf.py -p /dev/ttyUSB0 monitor
```

Or use a Python serial reader. Place the coolbag in the expected tire region. Expected UART output contains JSON like:

```json
{"ts":12345,"ta":26.2,"outside":-10.1,"center":-11.8,"inside":-9.4,"detected":true,"pixels":156}
```

All three temperatures should be in the -10 to -19 °C range (within a few degrees).

- [ ] **Step 3: Move target to simulate steering**

Slide the coolbag left and right across the FOV. Expected behavior:
- `detected` stays `true` while the target is in frame.
- The three averages shift; one segment becomes colder as the target moves into it.
- No reboots or exceptions.

- [ ] **Step 4: Remove target and verify missing-tire handling**

Point the sensor at a uniform wall. Expected output:

```json
{"ts":12390,"ta":26.3,"detected":false,"pixels":0}
```

- [ ] **Step 5: Commit test notes (optional)**

If any tunables need adjustment (e.g., `TIRE_THRESHOLD_OFFSET`), edit `main/tire_segment.c` and commit:

```bash
git add main/tire_segment.c
git commit -m "tune(tire_segment): adjust threshold after HIL test"
```

---

## Self-review

### Spec coverage

| Spec section | Task implementing it |
|--------------|----------------------|
| Temperature threshold (25th percentile + offset) | Task 2 |
| Connected component (largest blob) | Task 2 |
| Principal axis | Task 2 |
| Equal-count split into outside/center/inside | Task 2 |
| Outside = left/top | Task 2 (most-negative projection band) |
| JSON output format | Task 2 + Task 3 |
| Error handling (detected false) | Task 2 + Task 3 |
| 1 Hz refresh / low latency | Task 3 loop timing |
| Hardware-in-the-loop testing | Task 5 |

### Placeholder scan

- No TBD/TODO entries.
- All code blocks contain complete function bodies.
- All commands include expected output.
- No vague "handle edge cases" steps.

### Type consistency

- `tire_segment_process` signature matches `tire_segment.h`.
- `tire_segment_json` signature matches `tire_segment.h`.
- `mqttcomm_publish` topic matches existing convention.

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-26-tire-segmentation.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using `executing-plans`, batch execution with checkpoints.

Which approach?
