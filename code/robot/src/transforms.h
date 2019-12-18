#ifndef _TRANSFORMS_
#define _TRANSFORMS_

#include <dataStructs.h>
#include <inttypes.h>

#define TICKS_TO_RAD 0.0218166


void torque_conversion(MATRIX* M_torques, TORQUES* T_real, TORQUES* T_virtual);

void read_IMU(ANGLES* IMUangles, float pitch, float roll, float yaw);

void read_enc(volatile ANGLES* omniangles, int32_t enc1, int32_t enc2, int32_t enc3);

#endif