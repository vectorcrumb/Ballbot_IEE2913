#ifndef _DATASTRUCTS_
#define _DATASTRUCTS_
//For K, x0, x, deltax
typedef struct state_data{
    float phix = 0;
    float thetax = 0;
    float dphix = 0;
    float dthetax = 0;
    float phiy = 0;
    float thetay = 0;
    float dphiy = 0;
    float dthetay = 0;
    float thetaz = 0;
    float dthetaz = 0;
    float Thetax_offset = 0;
    float Thetay_offset = 0;
    float Thetaz_offset = 0;
} STATE_DATA;

//T_virtual, T_real, deltau, u0
typedef struct Torques{
    float Tx1 = 0;
    float Ty2 = 0;
    float Tz3 = 0;
} TORQUES;

//M_torques
typedef struct matrix{
    float m11 = 0;
    float m12 = 0;
    float m13 = 0;
    float m21 = 0;
    float m22 = 0;
    float m23 = 0;
    float m31 = 0;
    float m32 = 0;
    float m33 = 0;
} MATRIX;

//omniangles, IMUangles
typedef struct angles{
    float w1 = 0;
    float w2 = 0;
    float w3 = 0;
    float dw1 = 0;
    float dw2 = 0;
    float dw3 = 0;
} ANGLES;

//Voltages for each motor, PWM
typedef struct voltages{
    float V1 = 0;
    float V2 = 0;
    float V3 = 0;
} VOLTAGES;

#endif