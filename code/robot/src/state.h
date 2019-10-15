#ifndef _DATASTRUCTS_
#define _DATASTRUCTS_
#include <dataStructs.h>
#endif

void get_phi(STATE_DATA* deltax, STATE_DATA x0, MATRIX M_od_omniangles, MATRIX M_od_IMUangles, ANGLES omniangles, ANGLES IMUangles, float deltat);

void get_theta(STATE_DATA* deltax, ANGLES IMUangles);