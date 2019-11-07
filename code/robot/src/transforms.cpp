#include "transforms.h"


void torque_conversion(Mat33 * M_torques, Torque * T_real, Torque * T_virtual) {
    T_real->Tx1 = clamp_value((M_torques->m11)*(T_virtual->Tx1) + 
                            (M_torques->m12)*(T_virtual->Ty2) + 
                            (M_torques->m13)*(T_virtual->Tz3), MAX_TORQUE);

    T_real->Ty2 = clamp_value((M_torques->m21)*(T_virtual->Tx1) + 
                            (M_torques->m22)*(T_virtual->Ty2) + 
                            (M_torques->m23)*(T_virtual->Tz3), MAX_TORQUE);

    T_real->Tz3 = clamp_value((M_torques->m31)*(T_virtual->Tx1) + 
                            (M_torques->m32)*(T_virtual->Ty2) + 
                            (M_torques->m33)*(T_virtual->Tz3), MAX_TORQUE);
}

/**
 * Stores IMU readings and computes a discrete derivative for angular speed
 */
void read_IMU(AngleState * imu_angles, float pitch, float roll, float yaw, uint32_t deltat) {
    // NOT NED ANGLES.
    imu_angles->dw1 = newton_derivative(IMU_X_MOD * pitch, imu_angles->w1, (deltat / 1000));
    imu_angles->dw2 = newton_derivative(IMU_Y_MOD * roll, imu_angles->w2, (deltat / 1000));
    imu_angles->dw3 = newton_derivative(IMU_Z_MOD * yaw, imu_angles->w3, (deltat / 1000));

    imu_angles->w1 = IMU_X_MOD * pitch;
    imu_angles->w2 = IMU_Y_MOD * roll;
    imu_angles->w3 = IMU_Z_MOD * yaw;
}

/**
 * Stores encoder readings and computes a discrete derivative for wheel speed
 */
void read_enc(AngleState * omniangles, int32_t enc1_count, int32_t enc2_count, int32_t enc3_count, uint32_t deltat){
    omniangles->dw1 = ((float)enc1_count * ENC_TO_ANGLE - omniangles->w1) / (deltat / 1000);
    omniangles->dw2 = ((float)enc2_count * ENC_TO_ANGLE - omniangles->w2) / (deltat / 1000);
    omniangles->dw3 = ((float)enc3_count * ENC_TO_ANGLE - omniangles->w3) / (deltat / 1000);
    omniangles->w1 = enc1_count * ENC_TO_ANGLE;
    omniangles->w2 = enc2_count * ENC_TO_ANGLE;
    omniangles->w3 = enc3_count * ENC_TO_ANGLE;
}