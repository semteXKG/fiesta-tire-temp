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
