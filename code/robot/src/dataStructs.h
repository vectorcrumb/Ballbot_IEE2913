#ifndef _DATASTRUCTS_
#define _DATASTRUCTS_

//For K, x0, x, deltax
typedef struct state {
    float phix;
    float thetax;
    float dphix;
    float dthetax;

    float phiy;
    float thetay;
    float dphiy;
    float dthetay;

    float thetaz;
    float dthetaz;

    float Thetax_offset;
    float Thetay_offset;
    float Thetaz_offset;
} State;

//T_virtual, T_real, deltau, u0
typedef struct torque {
    float Tx1;
    float Ty2;
    float Tz3;
} Torque;

//M_torques
typedef struct mat33 {
    float m11;
    float m12;
    float m13;

    float m21;
    float m22;
    float m23;

    float m31;
    float m32;
    float m33;
} Mat33;

//omniangles, IMUangles
typedef struct angleState {
    float w1;
    float w2;
    float w3;

    float dw1;
    float dw2;
    float dw3;
} AngleState;

//Voltages for each motor, PWM
typedef struct motorSignal {
    float V1;
    float V2;
    float V3;
} MotorSignal;

#endif