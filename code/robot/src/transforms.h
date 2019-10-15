#ifndef _DATASTRUCTS_
#define _DATASTRUCTS_
#include <dataStructs.h>
#endif

void torque_conversion(MATRIX M_torques, TORQUES* T_real, TORQUES T_virtual);

void get_opPoint(MATRIX* M_torques, STATE_DATA* K, STATE_DATA* x0, TORQUES* u0, int opPoint_number);