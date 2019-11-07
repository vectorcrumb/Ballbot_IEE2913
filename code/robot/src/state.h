#ifndef _STATE_H
#define _STATE_H

#include <inttypes.h>
#include "dataStructs.h"

// Defines are for default values
#define K1 0.0048
#define K2 8.500
#define K3 0.1575
#define K4 3.1767

void set_theta_offset(State * deltax, float x_offset, float y_offset, float z_offset);
void get_phi(State * deltax, State * x0, 
            Mat33 * M_od_omniangles, Mat33 * M_od_IMUangles, 
            AngleState * omniangles, AngleState * IMUangles, 
            uint32_t deltat);
void get_theta(State * deltax, AngleState * IMUangles, State * x0);
void get_opPoint(Mat33 * M_odometry_omniangles, 
                Mat33 * M_odometry_IMUangles, 
                Mat33 * M_torques, 
                State * K, 
                State * x0, 
                Torque * u0, 
                uint16_t op_idx);
                
#endif