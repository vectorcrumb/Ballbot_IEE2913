#include "transforms.h"


void torque_conversion(MATRIX* M_torques, TORQUES* T_real, TORQUES* T_virtual){
    T_real->Tx1 = (M_torques->m11)*(T_virtual->Tx1)+(M_torques->m12)*(T_virtual->Ty2)+(M_torques->m13)*(T_virtual->Tz3);
    T_real->Ty2 = (M_torques->m21)*(T_virtual->Tx1)+(M_torques->m22)*(T_virtual->Ty2)+(M_torques->m23)*(T_virtual->Tz3);
    T_real->Tz3 = (M_torques->m31)*(T_virtual->Tx1)+(M_torques->m32)*(T_virtual->Ty2)+(M_torques->m33)*(T_virtual->Tz3);
    if (T_real->Tx1>2.5) T_real->Tx1 =2.5;
    if (T_real->Ty2>2.5) T_real->Ty2 =2.5;
    if (T_real->Tz3>2.5) T_real->Tz3 =2.5;
    if (T_real->Tx1<-2.5) T_real->Tx1 =-2.5;
    if (T_real->Ty2<-2.5) T_real->Ty2 =-2.5;
    if (T_real->Tz3<-2.5) T_real->Tz3 =-2.5;
}

void read_IMU(volatile ANGLES* IMUangles, float pitch, float roll, float yaw) {

    IMUangles->dw1 = (pitch - IMUangles->w1) * 100;
    IMUangles->dw2 = (roll - IMUangles->w2) * 100;
    IMUangles->dw3 = (yaw - IMUangles->w3) * 100;
    IMUangles->w1 = pitch;
    IMUangles->w2 = roll;
    IMUangles->w3 = yaw;
}

void read_enc(volatile ANGLES* omniangles, int32_t enc1, int32_t enc2, int32_t enc3){

    omniangles->dw1 = ((float)enc1 * TICKS_TO_RAD - omniangles->w1) * 100;
    omniangles->dw2 = ((float)enc2 * TICKS_TO_RAD - omniangles->w2) * 100;
    omniangles->dw3 = ((float)enc3 * TICKS_TO_RAD - omniangles->w3) * 100;
    omniangles->w1 = (float) enc1 * TICKS_TO_RAD;
    omniangles->w2 = (float) enc2 * TICKS_TO_RAD;
    omniangles->w3 = (float) enc3 * TICKS_TO_RAD;
}