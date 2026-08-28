#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"

void telemetry_print(const char *message);

extern UART_HandleTypeDef huart2;

#endif