#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;   // TIM in encoder mode
    float ticks_per_rev;
    float drum_diameter_in;
    volatile int32_t count;
    volatile float distance_in;
    volatile uint8_t reset_request;
} RotaryEncoder_t;

// Initialize module
void RE_Init(RotaryEncoder_t *enc,
             TIM_HandleTypeDef *htim,
             float ticks_per_rev,
             float drum_diameter_in);

// Must be called often in main loop
void RE_Update(RotaryEncoder_t *enc);

// EXTI callback hook for SW button
void RE_ButtonPressed(RotaryEncoder_t *enc);

// Access functions
int32_t RE_GetCount(RotaryEncoder_t *enc);
float RE_GetRevolutions(RotaryEncoder_t *enc);
float RE_GetDistanceInches(RotaryEncoder_t *enc);
void RE_Reset(RotaryEncoder_t *enc);

#endif
