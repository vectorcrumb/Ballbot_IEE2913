#include "control.h"
#include <math.h>

void control_signal(TORQUES* T_virtual, STATE_DATA* K, TORQUES* u0, STATE_DATA* deltax){
    float T_x = -(K->phix)*(deltax->phix)-(K->dphix)*(deltax->dphix)-(K->thetax)*(deltax->thetax)-(K->dthetax)*(deltax->dthetax);
    float T_y = -(-(K->phiy)*(deltax->phiy)-(K->dphiy)*(deltax->dphiy)-(K->thetay)*(deltax->thetay)-(K->dthetay)*(deltax->dthetay));
    float T_z = -(K->thetaz)*(deltax->thetaz)-(K->dthetaz)*(deltax->dthetaz);
    T_virtual->Tx1 = T_x + (u0->Tx1);
    T_virtual->Ty2 = T_y + (u0->Ty2);
    T_virtual->Tz3 = T_z + (u0->Tz3);
}

void voltage_motors(VOLTAGES* V_PWM, TORQUES* T_real, ANGLES* omniangles, float deltat){

    
}

void voltage_pwm(VOLTAGES* V, VOLTAGES* PWM, float V_battery){
    PWM->V1 = 255*(V->V1)/float(V_battery);
    PWM->V2 = 255*(V->V2)/float(V_battery);
    PWM->V3 = 255*(V->V3)/float(V_battery);
}