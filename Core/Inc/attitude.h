#ifndef ATTITUDE_H
#define ATTIUDE_H

#include "stm32f4xx_hal_def.h"
#include "quaternion.h"

HAL_StatusTypeDef attitude_update(Quaternion *current_attitude);
void init_t0(void);

#endif