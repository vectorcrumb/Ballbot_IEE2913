#ifndef _TRANSFORMS_
#define _TRANSFORMS_
#include <dataStructs.h>
#include <MPU9250.h>
#include <Encoder.h>

void torque_conversion(MATRIX* M_torques, TORQUES* T_real, TORQUES* T_virtual);

void read_IMU(ANGLES* IMUangles, MPU9250 imu, float deltat);

void read_enc(volatile ANGLES* omniangles, int32_t enc1, int32_t enc2, int32_t enc3, uint32_t deltat);

#endif