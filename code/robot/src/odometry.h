#include "Arduino.h"

typedef struct angles{
    int angle_x;
    int angle_y;
    int angle_z;
} ANGLES;

typedef struct encoders{
    int enc_1;
    int enc_2;
    int enc_3;
} ENCODERS;

void read_encoders(ENCODERS* encoder_angles, int enc_1, int enc_2, int enc_3);

void odometry(ANGLES* real_angles, ENCODERS encoder_angles);