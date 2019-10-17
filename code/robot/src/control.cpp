#include "control.h"
#include <math.h>

void control_signal(TORQUES* T_virtual, STATE_DATA* K, TORQUES* u0, STATE_DATA* deltax){
    float T_x = -(K->phix)*(deltax->phix)-(K->dphix)*(deltax->dphix)-(K->thetax)*(deltax->thetax)-(K->dthetax)*(deltax->dthetax);
    float T_y = -(K->phiy)*(deltax->phiy)-(K->dphiy)*(deltax->dphiy)-(K->thetay)*(deltax->thetay)-(K->dthetay)*(deltax->dthetay);
    float T_z = -(K->thetaz)*(deltax->thetaz)-(K->dthetaz)*(deltax->dthetaz);
    T_virtual->Tx1 = T_x + (u0->Tx1);
    T_virtual->Ty2 = T_y + (u0->Ty2);
    T_virtual->Tz3 = T_z + (u0->Tz3);
}

void voltage_motors(VOLTAGES* V, TORQUES* T_real, ANGLES* omniangles, float deltat){
    float G=0.1653;
    float G_inv=6.0496;
    float R=2.7273;
    float J=0.0001317;
    float T1 = T_real->Tx1;
    float T2 = T_real->Ty2;
    float T3 = T_real->Tz3;
    float omega1 = (T_real->Tx1)*0.000001*deltat/J+(omniangles->dw1);
    float omega2 = (T_real->Ty2)*0.000001*deltat/J+(omniangles->dw2);
    float omega3 = (T_real->Tz3)*0.000001*deltat/J+(omniangles->dw3);
    int sg1 = 1; 
    int sg2 = 1; 
    int sg3 = 1; 
    if (T1<0) {T1 *= -1; omega1 *= -1; sg1*= -1;}
    if (T2<0) {T2 *= -1; omega2 *= -1; sg2*= -1;}
    if (T3<0) {T3 *= -1; omega3 *= -1; sg3*= -1;}
    V->V1 = sg1*sqrt(T1*G_inv)*(R+G*omega1);
    V->V2 = sg2*sqrt(T2*G_inv)*(R+G*omega2);
    V->V3 = sg3*sqrt(T3*G_inv)*(R+G*omega3);
}

void voltage_pwm(VOLTAGES* V, VOLTAGES* PWM, float V_battery){
    PWM->V1 = (V->V1)/V_battery;
    PWM->V2 = (V->V2)/V_battery;
    PWM->V3 = (V->V3)/V_battery;
}