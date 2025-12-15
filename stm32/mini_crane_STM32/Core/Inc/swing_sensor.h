#ifndef SWING_SENSOR_H
#define SWING_SENSOR_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

typedef struct {
    float angle;          // filtered angle
    float raw_angle;      // unfiltered angle
    float angle_offset;   // zero reference
    bool  offset_valid;
} SwingSensor_t;

extern SwingSensor_t SwingSensor;

// ---- API ----
void SwingSensor_Init(I2C_HandleTypeDef *hi2c);
void SwingSensor_Update(void);      // call every 10–20 ms
void SwingSensor_CalibrateZero(void);

float SwingSensor_GetAngle(void);   // filtered + zeroed
bool  SwingSensor_IsSwinging(void);

#endif
