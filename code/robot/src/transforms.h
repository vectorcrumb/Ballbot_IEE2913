#ifndef _TRANSFORMS_
#define _TRANSFORMS_
#include <dataStructs.h>
#include <MPU9250.h>
#include <Encoder.h>

void torque_conversion(MATRIX* M_torques, TORQUES* T_real, TORQUES* T_virtual);

void read_IMU(ANGLES* IMUangles, MPU9250 imu, float deltat);

void read_enc(ANGLES* omniangles, Encoder enc1, Encoder enc2, Encoder enc3, float deltat);

#endif