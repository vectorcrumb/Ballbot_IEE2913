#ifndef _TORQUE_MOTOR_H_
#define _TORQUE_MOTOR_H_

#include <inttypes.h>
#include "PID.h"
#include "VNHDriver.h"


/**
 * Kp constant approximated as V/e_max = Kp = 12.6/(2.8 - 0) = 4.5
 */
#define TORQUE_PID_KP 1.5
#define TORQUE_PID_KI 0.4
#define TORQUE_PID_KD 0
#define TORQUE_PID_MIN_U -6
#define TORQUE_PID_MAX_U 6
#define PID_DELTA_TIME 0.020

class TorqueMotor {
public:
    TorqueMotor(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin, uint8_t cs_pin, float kt);
    ~TorqueMotor();
    void begin();
    void setTorque(float torque);
    void updateMotor(float refresh_rate);
    float torque_setpoint, torque_measured, output;
private:
    PID * controller;
    VNHDriver * motor;
    uint8_t pwmPin, inaPin, inbPin, csPin;
    float Kt;
};

#endif