#include "telemetry.h"
#include <string.h>
#include <stdio.h>

void telemetry_print(const char *message) {
    // Transmit the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void telemetry_send_quaternion(Quaternion data) {
  char message[32];
  snprintf(message, sizeof(message), "%0.3f,%0.3f,%0.3f,%0.3f\r\n", data.w, data.x, data.y, data.z);
  telemetry_print(message);
}