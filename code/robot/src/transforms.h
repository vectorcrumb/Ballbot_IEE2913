#ifndef _TRANSFORMS_
#define _TRANSFORMS_

#include "dataStructs.h"
#include "util.h"


#define ENC_TO_ANGLE 0.0218166
#define MAX_TORQUE 2.8
#define IMU_X_MOD 1
#define IMU_Y_MOD 1
#define IMU_Z_MOD 1

void torque_conversion(Mat33 * M_torques, Torque * T_real, Torque * T_virtual);
void read_IMU(AngleState * imu_angles, float pitch, float roll, float yaw, uint32_t deltat);
void read_enc(AngleState * omniangles, int32_t enc1_count, int32_t enc2_count, int32_t enc3_count, uint32_t deltat);


#endif