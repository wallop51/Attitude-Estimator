#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"
#include "quaternion.h"

void telemetry_print(const char *message);
void telemetry_send_quaternion(Quaternion data);

extern UART_HandleTypeDef huart2;

#endif