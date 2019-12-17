#include "transforms.h"
#define TICKS_TO_RAD 0.0218166


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

void read_IMU(ANGLES* IMUangles, MPU9250 imu, float deltat){
    // NOT NED ANGLES.
    float ThetaX = -1*imu.pitch; 
    float ThetaY = imu.roll;
    float ThetaZ = imu.yaw*0;
    IMUangles->dw1 = (ThetaX-IMUangles->w1)/deltat*1000;
    IMUangles->dw2 = (ThetaY-IMUangles->w2)/deltat*1000;
    IMUangles->dw3 = (ThetaZ-IMUangles->w3)/deltat*1000;
    IMUangles->w1 = ThetaX;
    IMUangles->w2 = ThetaY;
    IMUangles->w3 = ThetaZ;
    
}

void read_enc(volatile ANGLES* omniangles, int32_t enc1, int32_t enc2, int32_t enc3){
    // omniangles->past_speeds1[omniangles->indice]=((float)enc1 * TICKS_TO_RAD - omniangles->w1) * 100;
    // omniangles->past_speeds2[omniangles->indice]=((float)enc2 * TICKS_TO_RAD - omniangles->w2) * 100;
    // omniangles->past_speeds3[omniangles->indice]=((float)enc3 * TICKS_TO_RAD - omniangles->w3) * 100;
    // omniangles->indice++;
    // if (omniangles->indice>4) omniangles->indice=0;
    // omniangles->dw1 = (omniangles->past_speeds1[0]+omniangles->past_speeds1[1]+omniangles->past_speeds1[2]+omniangles->past_speeds1[3]+omniangles->past_speeds1[4])*0.2;
    // omniangles->dw2 = (omniangles->past_speeds2[0]+omniangles->past_speeds2[1]+omniangles->past_speeds2[2]+omniangles->past_speeds2[3]+omniangles->past_speeds2[4])*0.2;
    // omniangles->dw3 = (omniangles->past_speeds3[0]+omniangles->past_speeds3[1]+omniangles->past_speeds3[2]+omniangles->past_speeds3[3]+omniangles->past_speeds3[4])*0.2;
    
    omniangles->dw1 = ((float)enc1 * TICKS_TO_RAD - omniangles->w1) * 100;
    omniangles->dw2 = ((float)enc2 * TICKS_TO_RAD - omniangles->w2) * 100;
    omniangles->dw3 = ((float)enc3 * TICKS_TO_RAD - omniangles->w3) * 100;
    omniangles->w1 = (float) enc1 * TICKS_TO_RAD;
    omniangles->w2 = (float) enc2 * TICKS_TO_RAD;
    omniangles->w3 = (float) enc3 * TICKS_TO_RAD;
}