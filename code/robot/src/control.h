#ifndef _DATASTRUCTS_
#define _DATASTRUCTS_
#include <dataStructs.h>
#endif

void control_signal(TORQUES* T_virtual, STATE_DATA K, TORQUES u0, STATE_DATA deltax);

void voltage_motors(VOLTAGES* V, TORQUES T_real, ANGLES omniangles, float deltat);

void voltage_pwm(VOLTAGES V, VOLTAGES* PWM, float V_battery);