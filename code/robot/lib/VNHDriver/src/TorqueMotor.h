#ifndef _TORQUE_MOTOR_H_
#define _TORQUE_MOTOR_H_

#include "VNHDriver.h"

// #define TIME_CONVERSION_MS 1000
// #define TIME_CONVERSION_US 1000000
// #define TIME_FACTOR TIME_CONVERSION_US
#define MOTOR_PID_DT 0.020
#define MOTOR_PID_DT_US 20000
#define MOTOR_PID_DT_INV 50
#define MOTOR_ADC_RESOLUTION 10
#define MOTOR_ADC_BITS 1024
#define MOTOR_KT 0.72
/** 
 * This value is used to stretch the input range from a small value to the whole ADC range.
 * For a 5V/5A current sensor, the readings are optimized for a +/- 925mV range around the
 * 0A center point (VCC/2 = 2.5V), because 5A @ 185mV/A gives 0.925V for 5A. The following 
 * value is calculated as 0.925[V] * 1024[bits] / 5[V], giving ~190 ADC bits for the full 
 * 5A range, swinging around the 512 bits center range. Values read by the ADC are optimized 
 * for the [322.56, 701.44] range, however values outside this range may still be read, 
 * albeit with nonlinearities in the readings. 
 * 
 * Substracting the center point from the reading gives values in the [-190, 190] range, 
 * indicating current with sign. The torque PID controller is interested in the torque
 * value read from the current sensor, so the reading should be normalized and later scaled
 * by the corresponding torque for a 5A reading, given by the EM law tau = Kt * I_a.
 */
#define MOTOR_SENSOR_BITS_RANGE 189.44
#define MOTOR_TORQUE_AT_AMP_LIMIT 3.6
// TORQUE_AT_AMP_LIMIT / SENSOR_BITS_RANGE
#define MOTOR_CURRENT_TO_TORQUE_FACTOR 0.019


// y = ax+b
#define CURRENT_A 0.00870988
#define CURRENT_B -13.5135
#define TORQUE_A 0.00627111
#define TORQUE_B -9.72973


/**
 * Kp constant approximated as DC/e_max = Kp = 1.0/(2.8 - 0) = 0.357
 */
#define MOTOR_PID_KP 0.25
#define MOTOR_PID_KI 0.8
#define MOTOR_PID_MIN_U -1
#define MOTOR_PID_MAX_U 1
#define MOTOR_PID_MAX_INT 2

class TorqueMotor {
public:
    TorqueMotor(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin, uint8_t cs_pin, bool inverted);
    ~TorqueMotor();
    void setTorque(float torque);
    void updateMotor(int16_t analog_reading);
    float calculatePID(float setpoint, float measurement);
    float getCurrent();
    float getTorque();
    float getError();
    void _setMotorSpeed(float speed);
    float torque_setpoint = 0, torque_measured = 0, output = 0;
    float err = 0, err_int = 0;
    float Kp = MOTOR_PID_KP, Ki = MOTOR_PID_KI;
    int16_t analog_measurement = 0;
    uint8_t csPin;
    float zeroPointCurrent = 0;
private:
    VNHDriver * motor;
    uint8_t pwmPin, inaPin, inbPin;
    int8_t direction = 1;
};

#endif