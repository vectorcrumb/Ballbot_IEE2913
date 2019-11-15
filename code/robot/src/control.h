#ifndef _CONTROL_
#define _CONTROL_

#include <math.h>
#include "dataStructs.h"
#include <Arduino.h>

void control_signal(Torque * T_virtual, State * K, Torque * u0, State * deltax);
void voltage_motors(MotorSignal * V, Torque * T_real, AngleState * omniangles, float deltat);
void voltage_pwm(MotorSignal * V, MotorSignal * PWM, float V_battery);

#endif