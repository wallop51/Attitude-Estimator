#ifndef ATTITUDE_H
#define ATTIUDE_H

#include "stm32f4xx_hal_def.h"

HAL_StatusTypeDef attitude_update(Quaternion *current_attitude);

#endif