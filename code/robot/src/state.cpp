#include "state.h"
#include "Arduino.h"

#define K1 0.0039
#define K2 9.7968
#define K3 0.1274
#define K4 3.4603

void get_phi(STATE_DATA* deltax, STATE_DATA* x0, MATRIX* M_od_omniangles, MATRIX* M_od_IMUangles, ANGLES* omniangles, ANGLES* IMUangles, float deltat){
    float dPhix = (M_od_omniangles->m11)*(omniangles->dw1)+(M_od_omniangles->m12)*(omniangles->dw2)+(M_od_omniangles->m13)*(omniangles->dw3)+(M_od_IMUangles->m11)*(IMUangles->dw1)+(M_od_IMUangles->m12)*(IMUangles->dw2)+(M_od_IMUangles->m13)*(IMUangles->dw3);
    float dPhiy = (M_od_omniangles->m21)*(omniangles->dw1)+(M_od_omniangles->m22)*(omniangles->dw2)+(M_od_omniangles->m23)*(omniangles->dw3)+(M_od_IMUangles->m21)*(IMUangles->dw1)+(M_od_IMUangles->m22)*(IMUangles->dw2)+(M_od_IMUangles->m23)*(IMUangles->dw3);
    float Phix = (x0->phix+deltax->phix)+deltat*0.000001*dPhix;
    float Phiy = (x0->phiy+deltax->phiy)+deltat*0.000001*dPhiy;
    deltax->dphix = dPhix-x0->dphix;
    deltax->dphiy = dPhiy-x0->dphiy;
    deltax->phix = Phix-x0->phix;
    deltax->phiy = Phiy-x0->phiy;
}

void get_theta(STATE_DATA* deltax, ANGLES* IMUangles, STATE_DATA* x0){
    deltax->thetax = (IMUangles->w1)-(x0->thetax)-(deltax->Thetax_offset);
    deltax->thetay = (IMUangles->w2)-(x0->thetay)-(deltax->Thetay_offset);
    deltax->thetaz = (IMUangles->w3)-(x0->thetaz)-(deltax->Thetaz_offset)*0;
    deltax->dthetax = (IMUangles->dw1)-(x0->dthetax);
    deltax->dthetay = (IMUangles->dw2)-(x0->dthetay);
    deltax->dthetaz = (IMUangles->dw3)-(x0->dthetaz);
}

void get_opPoint(MATRIX* M_odometry_omniangles, MATRIX* M_odometry_IMUangles, MATRIX* M_torques, STATE_DATA* K, STATE_DATA* x0, TORQUES* u0, int opPoint_number){
    //POR ESCRIBIR
    //leer de SD en base a el punto de operación

    K->phix=K1;
    K->thetax=K2;
    K->dphix=K3;
    K->dthetax=K4;
    K->phiy=K1;
    K->thetay=K2;
    K->dphiy=K3;
    K->dthetay=K4;
    K->thetaz=4.5175;
    K->dthetaz=0.9804;
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