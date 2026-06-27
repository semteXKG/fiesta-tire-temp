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
    bool detected;          /**< true when a tire-sized region is found */
    uint16_t pixels;        /**< number of pixels in the detected region */
    float outside;          /**< average temperature of the outside third (°C) */
    float center;           /**< average temperature of the center third (°C) */
    float inside;           /**< average temperature of the inside third (°C) */
    float ta;               /**< ambient temperature used for segmentation (°C) */
    uint32_t timestamp_ms;  /**< timestamp in milliseconds, populated by the caller */
} tire_segment_result_t;

esp_err_t tire_segment_process(const float *temps, float ta, tire_segment_result_t *out);
int tire_segment_json(const tire_segment_result_t *r, char *buf, size_t buflen);
int tire_segment_raw_json(uint32_t timestamp_ms, float ta, const float *pixels, size_t n, char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* TIRE_SEGMENT_H */
