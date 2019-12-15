#include "TorqueMotor.h"


TorqueMotor::TorqueMotor(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin, uint8_t cs_pin) {
    // Configure motor
    this->motor = new VNHDriver();
    this->motor->begin(pwm_pin, ina_pin, inb_pin);
    this->setTorque(0);
    // Current sensor pin
    this->csPin = cs_pin;
}

TorqueMotor::~TorqueMotor() {
    delete this->motor;
}

void TorqueMotor::setTorque(float torque) {
    this->torque_setpoint = torque;
}

uint16_t TorqueMotor::getCurrent() {
    return analogRead(this->csPin);
}

float TorqueMotor::getTorque() {
    // float torque = TORQUE_AT_AMP_LIMIT * ((float) this->getCurrent() - (ADC_BITS >> 1)) / SENSOR_BITS_RANGE;
    return MOTOR_CURRENT_TO_TORQUE_FACTOR * ((float) this->getCurrent() - (MOTOR_ADC_BITS >> 1));
}

float TorqueMotor::calculatePID(float setpoint, float measurement) {
    this->err = setpoint - measurement;
    this->err_int += this->err * MOTOR_PID_DT;

    float pOut = this->Kp * this->err;
    float iOut = this->Ki * this->err_int;

    float u = pOut + iOut;
    if (u > MOTOR_PID_MAX_U) u = MOTOR_PID_MAX_U;
    if (u < MOTOR_PID_MIN_U) u = MOTOR_PID_MIN_U;

    return u;
}

/**
 * This function must be called periodically to succesfully update the PID controller. The function
 * first measures the torque via T = K_t * I, where I is read from the current sensor pin. It then
 * updates the PID controller, passing the torque setpoint and measurement (ref and y) and dt. Finally,
 * the PID controller returns a control signal in volts, which must then be scaled by the actuator range
 * (in this case, 12 - 0) to obtain a value between -1 and 1. This value scales 255 to control the motor.
 */
void TorqueMotor::updateMotor() {
    this->torque_measured = this->getTorque();
    this->output = this->calculatePID(this->torque_setpoint, this->torque_measured);
    this->motor->setSpeed(this->output);
}
