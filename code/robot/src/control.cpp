#include "control.h"
#include <math.h>

void control_signal(TORQUES* T_virtual, STATE_DATA K, TORQUES u0, STATE_DATA deltax){
    float T_x = -K.phix*deltax.phix-K.dphix*deltax.dphix-K.thetax*deltax.thetax-K.dthetax*deltax.dthetax;
    float T_y = -K.phiy*deltax.phiy-K.dphiy*deltax.dphiy-K.thetay*deltax.thetay-K.dthetay*deltax.dthetay;
    float T_z = -K.thetaz*deltax.thetaz-K.dthetaz*deltax.dthetaz;
    T_virtual->Tx1 = T_x + u0.Tx1;
    T_virtual->Ty2 = T_y + u0.Ty2;
    T_virtual->Tz3 = T_z + u0.Tz3;
}

void voltage_motors(VOLTAGES* V, TORQUES T_real, ANGLES omniangles, float deltat){
    float G=0.1653;
    float R=2.7273;
    float J=0.0001317;
    V->V1 = sqrt(T_real.Tx1/G)*(R+G*(T_real.Tx1*deltat/J+omniangles.w1));
    V->V2 = sqrt(T_real.Ty2/G)*(R+G*(T_real.Ty2*deltat/J+omniangles.w2));
    V->V3 = sqrt(T_real.Tz3/G)*(R+G*(T_real.Tz3*deltat/J+omniangles.w3));
}

void voltage_pwm(VOLTAGES V, VOLTAGES* PWM, float V_battery){
    PWM->V1 = V.V1/V_battery;
    PWM->V2 = V.V2/V_battery;
    PWM->V3 = V.V3/V_battery;
}