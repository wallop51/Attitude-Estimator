#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"

void Telemetry_Print(const char *message);

extern UART_HandleTypeDef huart2;

#endif