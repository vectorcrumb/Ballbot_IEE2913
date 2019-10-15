#include "transforms.h"

void torque_conversion(MATRIX M_torques, TORQUES* T_real, TORQUES T_virtual){
    T_real->Tx1 = M_torques.m11*T_virtual.Tx1+M_torques.m12*T_virtual.Ty2+M_torques.m13*T_virtual.Tz3;
    T_real->Ty2 = M_torques.m21*T_virtual.Tx1+M_torques.m22*T_virtual.Ty2+M_torques.m23*T_virtual.Tz3;
    T_real->Tz3 = M_torques.m31*T_virtual.Tx1+M_torques.m32*T_virtual.Ty2+M_torques.m33*T_virtual.Tz3;
}

void get_opPoint(MATRIX* M_odometry_omniangles, MATRIX* M_odometry_IMUangles, MATRIX* M_torques, STATE_DATA* K, STATE_DATA* x0, TORQUES* u0, int opPoint_number){
    //POR ESCRIBIR
    //leer de SD en base a el punto de operación
    K->phix=0.3765;
    K->thetax=9.7798;
    K->dphix=0.3939;
    K->dthetax=1.6841;
    K->phiy=0.3765;
    K->thetay=9.7798;
    K->dphiy=0.3939;
    K->dthetay=1.6841;
    K->thetaz=4.5175;
    K->dthetaz=0.9910;
    M_torques->m11=0.9428;
    M_torques->m12=0;
    M_torques->m13=-0.3333;
    M_torques->m21=-0.4714;
    M_torques->m22=-0.8165;
    M_torques->m23=-0.3333;
    M_torques->m31=-0.4714;
    M_torques->m32=0.8165;
    M_torques->m33=-0.3333;
    M_odometry_omniangles->m11=-0.9428;
    M_odometry_omniangles->m12=0.4714;
    M_odometry_omniangles->m13=0.4714;
    M_odometry_omniangles->m21=0;
    M_odometry_omniangles->m22=-0.8165;
    M_odometry_omniangles->m23=0.8165;
    M_odometry_omniangles->m31=0.4714;
    M_odometry_omniangles->m32=0.4714;
    M_odometry_omniangles->m33=0.4714;
    M_odometry_IMUangles->m11=1;
    M_odometry_IMUangles->m12=0;
    M_odometry_IMUangles->m13=0;
    M_odometry_IMUangles->m21=0;
    M_odometry_IMUangles->m22=1;
    M_odometry_IMUangles->m23=0;
    M_odometry_IMUangles->m31=0;
    M_odometry_IMUangles->m32=0;
    M_odometry_IMUangles->m33=1;
    x0->phix=0;
    x0->thetax=0;
    x0->dphix=0;
    x0->dthetax=0;
    x0->phiy=0;
    x0->thetay=0;
    x0->dphiy=0;
    x0->dthetay=0;
    x0->thetaz=0;
    x0->dthetaz=0;
    u0->Tx1=0;
    u0->Ty2=0;
    u0->Tz3=0;
}