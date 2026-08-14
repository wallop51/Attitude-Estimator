#include "telemetry.h"
#include <string.h>

void Telemetry_Print(const char *message) {
    // Transmit the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}