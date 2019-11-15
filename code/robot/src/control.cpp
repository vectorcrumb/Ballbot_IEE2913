#include "control.h"

void control_signal(Torque * T_virtual, State * K, Torque * u0, State * deltax){
    // Serial.print("deltax0"); Serial.println(deltax->phix);
    // Serial.print("deltax1"); Serial.println(deltax->dphix);
    // Serial.print("deltax2"); Serial.println(deltax->thetax);
    // Serial.print("deltax3"); Serial.println(deltax->dthetax);

    T_virtual->Tx1 = u0->Tx1 -(K->phix * deltax->phix + K->dphix * deltax->dphix + K->thetax * deltax->thetax + K->dthetax * deltax->dthetax);
    T_virtual->Ty2 = u0->Ty2 -(K->phiy * deltax->phiy + K->dphiy * deltax->dphiy + K->thetay * deltax->thetay + K->dthetay * deltax->dthetay);
    T_virtual->Tz3 = u0->Tz3 -(K->thetaz * deltax->thetaz + K->dthetaz * deltax->dthetaz);
}

void voltage_motors(MotorSignal * V, Torque * T_real, AngleState * omniangles, float deltat){
    float G=0.1653;
    float G_inv=6.0496;
    float R=2.7273;

    float Kt=0.7273;
    float Ktinv=1.3750;
    float Ke=0.9231;

    // V->V1 = T_real->Tx1*Ktinv*R + Ke*omniangles->dw1;
    // V->V2 = T_real->Ty2*Ktinv*R + Ke*omniangles->dw2;
    // V->V3 = T_real->Tz3*Ktinv*R + Ke*omniangles->dw3;

    V->V1 = sqrt(fabs(T_real->Tx1)*G_inv)*(R + G*omniangles->dw1*((T_real->Tx1 < 0) ? -1 : 1))*((T_real->Tx1 < 0) ? -1 : 1);
    V->V2 = sqrt(fabs(T_real->Ty2)*G_inv)*(R + G*omniangles->dw2*((T_real->Ty2 < 0) ? -1 : 1))*((T_real->Ty2 < 0) ? -1 : 1);
    V->V3 = sqrt(fabs(T_real->Tz3)*G_inv)*(R + G*omniangles->dw3*((T_real->Tz3 < 0) ? -1 : 1))*((T_real->Tz3 < 0) ? -1 : 1);
    //float J=0.0001317;
    // float T1 = T_real->Tx1;
    // float T2 = T_real->Ty2;
    // float T3 = T_real->Tz3;
    // // float omega1 = (T_real->Tx1)*0.000001*deltat/J+(omniangles->dw1);
    // // float omega2 = (T_real->Ty2)*0.000001*deltat/J+(omniangles->dw2);
    // // float omega3 = (T_real->Tz3)*0.000001*deltat/J+(omniangles->dw3);
    // float omega1 = omniangles->dw1;
    // float omega2 = omniangles->dw2;
    // float omega3 = omniangles->dw3;

    // int sg1 = 1;
    // int sg2 = 1; 
    // int sg3 = 1; 
    // if (T1<0) {T1 *= -1; omega1 *= -1; sg1*= -1;}
    // if (T2<0) {T2 *= -1; omega2 *= -1; sg2*= -1;}
    // if (T3<0) {T3 *= -1; omega3 *= -1; sg3*= -1;}
    // V->V1 = sg1*sqrt(T1*G_inv)*(R+G*omega1);
    // V->V2 = sg2*sqrt(T2*G_inv)*(R+G*omega2);
    // V->V3 = sg3*sqrt(T3*G_inv)*(R+G*omega3);
}

void voltage_pwm(MotorSignal * V, MotorSignal * PWM, float V_battery){
    PWM->V1 = (V->V1)/V_battery;
    PWM->V2 = (V->V2)/V_battery;
    PWM->V3 = (V->V3)/V_battery;
}