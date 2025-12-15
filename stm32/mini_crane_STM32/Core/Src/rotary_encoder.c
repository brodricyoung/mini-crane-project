#include "main.h"
#include "rotary_encoder.h"
#include <math.h>

// ------------------------------------------------------------
// Initialize encoder
// ------------------------------------------------------------
void RE_Init(RotaryEncoder_t *enc,
             TIM_HandleTypeDef *htim,
             float ticks_per_rev,
             float drum_diameter_in)
{
    enc->htim = htim;
    enc->ticks_per_rev = ticks_per_rev;
    enc->drum_diameter_in = drum_diameter_in;
    enc->count = 0;
    enc->distance_in = 0;
    enc->reset_request = 0;

    // Start encoder timer
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);

    // Reset hardware counter
    __disable_irq();
    enc->htim->Instance->CNT = 0;
    __enable_irq();
}

// ------------------------------------------------------------
// Call periodically from main loop (50–200 Hz)
// ------------------------------------------------------------
void RE_Update(RotaryEncoder_t *enc)
{
    // Handle reset request from EXTI
    if (enc->reset_request) {
        RE_Reset(enc);
        enc->reset_request = 0;
    }

    // Read hardware count
    enc->count = (int32_t)(enc->htim->Instance->CNT);

    float revs = enc->count / enc->ticks_per_rev;
    float circumference = (float)M_PI * enc->drum_diameter_in;

    enc->distance_in = revs * circumference;
}

// ------------------------------------------------------------
// External interrupt calls this (does NOT reset immediately)
// ------------------------------------------------------------
void RE_ButtonPressed(RotaryEncoder_t *enc)
{
    enc->reset_request = 1;
}

// ------------------------------------------------------------
// Reset everything cleanly
// ------------------------------------------------------------
void RE_Reset(RotaryEncoder_t *enc)
{
    __disable_irq();
    enc->htim->Instance->CNT = 0;   // reset hardware counter
    __enable_irq();

    enc->count = 0;
    enc->distance_in = 0.0f;
}

// ------------------------------------------------------------
// Accessor functions
// ------------------------------------------------------------
int32_t RE_GetCount(RotaryEncoder_t *enc)
{
    return enc->count;
}

float RE_GetRevolutions(RotaryEncoder_t *enc)
{
    return enc->count / enc->ticks_per_rev;
}

float RE_GetDistanceInches(RotaryEncoder_t *enc)
{
    return enc->distance_in;
}
