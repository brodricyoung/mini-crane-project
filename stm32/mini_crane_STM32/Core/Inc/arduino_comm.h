#ifndef ARDUINO_COMM_H
#define ARDUINO_COMM_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

#define ARDUINO_COMM_BUFFER_SIZE 50
#define ARDUINO_NUM_BUTTONS 6

typedef enum {
    BUTTON_PRESS,
    BUTTON_RELEASE
} ButtonEvent;

// Initialize Arduino communication
void ArduinoComm_Init(UART_HandleTypeDef *huart);

// To be called from HAL_UART_RxCpltCallback
void ArduinoComm_UART_RxCallback(UART_HandleTypeDef *huart);

// Check if a button event is available
bool ArduinoComm_GetButtonEvent(uint8_t *buttonId, ButtonEvent *event);

void CheckCompensationAndNotify(void);
void test_buzzer_uart(void);

#endif // ARDUINO_COMM_H
