#include "arduino_comm.h"
#include <string.h>
#include <stdio.h>

// --- Static variables ---
static UART_HandleTypeDef *arduinoUart;
extern UART_HandleTypeDef huart3;
extern bool compensation_in_progress;
extern bool prev_compensation_state;

static uint8_t rxByte;                           // Incoming byte
static char rxBuffer[ARDUINO_COMM_BUFFER_SIZE];  // Line buffer
static uint8_t idx = 0;

// Simple event queue (single event)
static bool eventAvailable = false;
static uint8_t eventButtonId;
static ButtonEvent eventType;

// --- Private function ---
static void parseCommand(const char *cmd) {
    // Expect format: BTN{n}_PRESS or BTN{n}_RELEASE
    if (strncmp(cmd, "BTN", 3) != 0) return;

    uint8_t btn = cmd[3] - '0'; // Button index 0-5
    if (btn >= ARDUINO_NUM_BUTTONS) return;

    if (strstr(cmd, "_PRESS") != NULL) {
        eventButtonId = btn;
        eventType = BUTTON_PRESS;
        eventAvailable = true;
    }
    else if (strstr(cmd, "_RELEASE") != NULL) {
        eventButtonId = btn;
        eventType = BUTTON_RELEASE;
        eventAvailable = true;
    }
}

// --- Public functions ---

void ArduinoComm_Init(UART_HandleTypeDef *huart) {
    arduinoUart = huart;
    idx = 0;
    eventAvailable = false;

    // Start UART reception interrupt
    HAL_UART_Receive_IT(arduinoUart, &rxByte, 1);
}

void ArduinoComm_UART_RxCallback(UART_HandleTypeDef *huart) {
    if (huart != arduinoUart) return;

    if (rxByte != '\n' && idx < ARDUINO_COMM_BUFFER_SIZE - 1) {
        rxBuffer[idx++] = rxByte;
    } else {
        rxBuffer[idx] = '\0';  // Null terminate
        parseCommand(rxBuffer);
        idx = 0;
    }

    // Re-enable UART interrupt
    HAL_UART_Receive_IT(arduinoUart, &rxByte, 1);
}

// Returns true if an event is ready, clears the event once read
bool ArduinoComm_GetButtonEvent(uint8_t *buttonId, ButtonEvent *event) {
    if (!eventAvailable) return false;

    *buttonId = eventButtonId;
    *event = eventType;
    eventAvailable = false;
    return true;
}


void CheckCompensationAndNotify(void)
{
    // If state changed
    if (compensation_in_progress != prev_compensation_state)
    {
        if (compensation_in_progress)
        {
            char msg[] = "BUZZ_ON\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
        }
        else
        {
            char msg[] = "BUZZ_OFF\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
        }

        prev_compensation_state = compensation_in_progress;
    }
}

void test_buzzer_uart(void)
{
    char on[] = "BUZZ_ON\n";
    char off[] = "BUZZ_OFF\n";

    // Send ON
    HAL_UART_Transmit(&huart3, (uint8_t*)on, strlen(on), 10);
    HAL_Delay(1000);

    // Send OFF
    HAL_UART_Transmit(&huart3, (uint8_t*)off, strlen(off), 10);
    HAL_Delay(1000);
}
