#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==== ENUMS ====
typedef enum {
    STOP = 0,
    FORWARD,
    REVERSE
} DCMotorDirection;

typedef enum {
    HOLD = 0,
    CLOCKWISE,
    COUNTERCLOCKWISE
} StepperMotorDirection;

// ==== PUBLIC API ====

// DC motors
void DCMotor_Init(void);
void WinchMotor_Set(DCMotorDirection direction);
void TrolleyMotor_Set(DCMotorDirection direction);

// Stepper motor
void SlewMotor_Init(void);
void SlewMotor_Set(float rpm, StepperMotorDirection dir);
void SlewMotor_Coast(void);

// Called from TIM5 interrupt (HAL_TIM_PeriodElapsedCallback)
void Stepper_Tick(void);
void Stepper_Timer_Callback(void);

// Globals needed by main.c (optional)
extern TIM_HandleTypeDef htim5;

#endif
