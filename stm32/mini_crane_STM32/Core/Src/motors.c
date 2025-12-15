#include "motors.h"

// ==== STATIC DATA ====

static const uint8_t step_seq[4][4] = {
    {1,0,1,0},
    {1,0,0,1},
    {0,1,0,1},
    {0,1,1,0}
};

static volatile int step_index = 0;
static volatile int step_dir = 0;                 // 1=CW, -1=CCW, 0=hold
static volatile uint32_t step_interval_ms = 10;   // ms per step
static const int steps_per_rev = 200;
static uint8_t coast_flag = 0;

// ==== PRIVATE PROTOTYPES ====
static void Stepper_SetPins(int idx);

// ==== DC MOTOR API IMPLEMENTATION ====

void DCMotor_Init(void)
{
    WinchMotor_Set(STOP);
    TrolleyMotor_Set(STOP);
}

void WinchMotor_Set(DCMotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            HAL_GPIO_WritePin(Winch_IN2_GPIO_Port, Winch_IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Winch_IN1_GPIO_Port, Winch_IN1_Pin, GPIO_PIN_SET);
            break;

        case REVERSE:
            HAL_GPIO_WritePin(Winch_IN1_GPIO_Port, Winch_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Winch_IN2_GPIO_Port, Winch_IN2_Pin, GPIO_PIN_SET);
            break;

        case STOP:
        default:
            HAL_GPIO_WritePin(Winch_IN1_GPIO_Port, Winch_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Winch_IN2_GPIO_Port, Winch_IN2_Pin, GPIO_PIN_RESET);
            break;
    }
}

void TrolleyMotor_Set(DCMotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            HAL_GPIO_WritePin(Trolley_IN4_GPIO_Port, Trolley_IN4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Trolley_IN3_GPIO_Port, Trolley_IN3_Pin, GPIO_PIN_SET);
            break;

        case REVERSE:
            HAL_GPIO_WritePin(Trolley_IN3_GPIO_Port, Trolley_IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Trolley_IN4_GPIO_Port, Trolley_IN4_Pin, GPIO_PIN_SET);
            break;

        case STOP:
        default:
            HAL_GPIO_WritePin(Trolley_IN3_GPIO_Port, Trolley_IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Trolley_IN4_GPIO_Port, Trolley_IN4_Pin, GPIO_PIN_RESET);
            break;
    }
}

// ==== STEPPER MOTOR IMPLEMENTATION ====

void SlewMotor_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);

    // Reset all coils
    HAL_GPIO_WritePin(Slew_IN1_GPIO_Port, Slew_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Slew_IN2_GPIO_Port, Slew_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Slew_IN3_GPIO_Port, Slew_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Slew_IN4_GPIO_Port, Slew_IN4_Pin, GPIO_PIN_RESET);

    SlewMotor_Coast();
}

static void Stepper_SetPins(int idx)
{
    HAL_GPIO_WritePin(Slew_IN1_GPIO_Port, Slew_IN1_Pin, step_seq[idx][0]);
    HAL_GPIO_WritePin(Slew_IN2_GPIO_Port, Slew_IN2_Pin, step_seq[idx][1]);
    HAL_GPIO_WritePin(Slew_IN3_GPIO_Port, Slew_IN3_Pin, step_seq[idx][2]);
    HAL_GPIO_WritePin(Slew_IN4_GPIO_Port, Slew_IN4_Pin, step_seq[idx][3]);
}

void SlewMotor_Set(float rpm, StepperMotorDirection dir)
{
    if (dir == HOLD || rpm <= 0.0f) {
        step_dir = 0;
    } else {
        step_dir = (dir == CLOCKWISE) ? 1 : -1;
    }

    if (rpm > 0.0f) {
        step_interval_ms = (uint32_t)(60000.0f / (rpm * steps_per_rev));
    }
}

void SlewMotor_Coast(void)
{
    coast_flag = 1;
}

void Stepper_Tick(void)
{
    if (step_dir == 0)
    {
        if (coast_flag) {
            HAL_GPIO_WritePin(Slew_IN1_GPIO_Port, Slew_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Slew_IN2_GPIO_Port, Slew_IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Slew_IN3_GPIO_Port, Slew_IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Slew_IN4_GPIO_Port, Slew_IN4_Pin, GPIO_PIN_RESET);
        } else {
            Stepper_SetPins(step_index);
        }
        return;
    }

    // Normal step sequence
    if (step_dir > 0)
        step_index = (step_index + 1) % 4;
    else
        step_index = (step_index + 3) % 4;

    Stepper_SetPins(step_index);
}

// ==== TIMER CALLBACK HOOK ====
void Stepper_Timer_Callback(void) {
    static uint32_t counter = 0;
    counter++;

    if (counter >= step_interval_ms)
    {
        Stepper_Tick();
        counter = 0;
    }
}

