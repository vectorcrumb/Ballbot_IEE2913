#include "transforms.h"
#define ENC_TO_ANGLE 0.0218166


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
    IMUangles->dw1 = (ThetaX-IMUangles->w1)/deltat*0.000001;
    IMUangles->dw2 = (ThetaY-IMUangles->w2)/deltat*0.000001;
    IMUangles->dw3 = (ThetaZ-IMUangles->w3)/deltat*0.000001;
    IMUangles->w1 = ThetaX;
    IMUangles->w2 = ThetaY;
    IMUangles->w3 = ThetaZ;
    
}

void read_enc(ANGLES* omniangles, Encoder enc1, Encoder enc2, Encoder enc3, float deltat){
    float Phi1 = enc1.read()*ENC_TO_ANGLE;
    float Phi2 = enc2.read()*ENC_TO_ANGLE;
    float Phi3 = enc3.read()*ENC_TO_ANGLE;
    float Phi1prima = ((Phi1-omniangles->w1)/deltat)*0.000001;
    float Phi2prima = ((Phi2-omniangles->w2)/deltat)*0.000001;
    float Phi3prima = ((Phi3-omniangles->w3)/deltat)*0.000001;
    omniangles->ddw1 = ((Phi1prima-omniangles->dw1)/deltat)*0.000001;
    omniangles->ddw2 = ((Phi2prima-omniangles->dw2)/deltat)*0.000001;
    omniangles->ddw3 = ((Phi3prima-omniangles->dw3)/deltat)*0.000001;
    omniangles->dw1 = Phi1prima;
    omniangles->dw2 = Phi2prima;
    omniangles->dw3 = Phi3prima;
    omniangles->w1 = Phi1;
    omniangles->w2 = Phi2;
    omniangles->w3 = Phi3;
}