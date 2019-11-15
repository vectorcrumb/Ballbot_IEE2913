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
    imu_angles->dw1 = ((float) IMU_X_MOD * pitch - imu_angles->w1) / ((float)deltat / 1000);
    imu_angles->dw2 = ((float) IMU_Y_MOD * roll  - imu_angles->w2) / ((float)deltat / 1000);
    imu_angles->dw3 = ((float) IMU_Z_MOD * yaw   - imu_angles->w3) / ((float)deltat / 1000);

    imu_angles->w1 = IMU_X_MOD * pitch;
    imu_angles->w2 = IMU_Y_MOD * roll;
    imu_angles->w3 = IMU_Z_MOD * yaw;

    // Serial.print("imu_dt: "); Serial.println(deltat);
    // Serial.print("imu_w1: "); Serial.println(imu_angles->w1);
    // Serial.print("imu_w2: "); Serial.println(imu_angles->w2);
    // Serial.print("imu_w3: "); Serial.println(imu_angles->w3);
    // Serial.print("imu_dw1: "); Serial.println(imu_angles->dw1);
    // Serial.print("imu_dw2: "); Serial.println(imu_angles->dw2);
    // Serial.print("imu_dw3: "); Serial.println(imu_angles->dw3);
}

/**
 * Stores encoder readings and computes a discrete derivative for wheel speed
 */
void read_enc(AngleState * omniangles, int32_t enc1_count, int32_t enc2_count, int32_t enc3_count, uint32_t deltat){
    // Serial.print("omni_enc1: ");  Serial.println(enc1_count);
    // Serial.print("omni_enc2: ");  Serial.println(enc2_count);
    // Serial.print("omni_enc3: ");  Serial.println(enc3_count);
    // Serial.print("omni_aw1: ");  Serial.println(omniangles->w1);
    // Serial.print("omni_aw2: ");  Serial.println(omniangles->w2);
    // Serial.print("omni_aw3: ");  Serial.println(omniangles->w3);

    omniangles->dw1 = ((float)enc1_count * ENC_TO_ANGLE - omniangles->w1) / ((float)deltat / 1000);
    omniangles->dw2 = ((float)enc2_count * ENC_TO_ANGLE - omniangles->w2) / ((float)deltat / 1000);
    omniangles->dw3 = ((float)enc3_count * ENC_TO_ANGLE - omniangles->w3) / ((float)deltat / 1000);
    omniangles->w1 = enc1_count * ENC_TO_ANGLE;
    omniangles->w2 = enc2_count * ENC_TO_ANGLE;
    omniangles->w3 = enc3_count * ENC_TO_ANGLE;

    // Serial.print("omni_dt: "); Serial.println(deltat);
    // Serial.print("omni_w1: ");  Serial.println(omniangles->w1);
    // Serial.print("omni_w2: ");  Serial.println(omniangles->w2);
    // Serial.print("omni_w3: ");  Serial.println(omniangles->w3);
    // Serial.print("omni_dw1: "); Serial.println(omniangles->dw1);
    // Serial.print("omni_dw2: "); Serial.println(omniangles->dw2);
    // Serial.print("omni_dw3: "); Serial.println(omniangles->dw3);
}