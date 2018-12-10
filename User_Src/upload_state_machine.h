#ifndef UPLOAD_STATE_MACHINE_H
#define UPLOAD_STATE_MACHINE_H

#include "stdint.h"
#include "IMU.h"
#include "UARTs.h"

void update_upload_state(uint8_t* p_state, const float ypr[3], int16_t* p_math_hz);
#endif
