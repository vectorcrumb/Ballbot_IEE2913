#ifndef _STATE_H
#define _STATE_H
#include <dataStructs.h>

void get_phi(STATE_DATA* deltax, STATE_DATA* x0, MATRIX* M_od_omniangles, MATRIX* M_od_IMUangles, ANGLES* omniangles, ANGLES* IMUangles, float deltat);

void get_theta(STATE_DATA* deltax, ANGLES* IMUangles, STATE_DATA* x0);

void get_opPoint(MATRIX* M_odometry_omniangles, MATRIX* M_odometry_IMUangles, MATRIX* M_torques, STATE_DATA* K, STATE_DATA* x0, TORQUES* u0, int opPoint_number);

#endif