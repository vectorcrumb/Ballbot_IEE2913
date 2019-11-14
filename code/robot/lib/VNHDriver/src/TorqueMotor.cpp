#include "TorqueMotor.h"


TorqueMotor::TorqueMotor(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin, uint8_t cs_pin, float kt) {
    this->motor = new VNHDriver();
    this->motor->begin(pwm_pin, ina_pin, inb_pin);
    this->setTorque(0);
    this->csPin = cs_pin;
    this->Kt = kt;
    this->torque_measured = 0;
    controller = new PID(TORQUE_PID_KP, TORQUE_PID_KI, TORQUE_PID_KD, TORQUE_PID_MIN_U, TORQUE_PID_MAX_U, PID_DELTA_TIME);
}

TorqueMotor::~TorqueMotor() {
    delete this->motor;
    delete this->controller;
}

void TorqueMotor::begin() {

}

void TorqueMotor::setTorque(float torque) {
    this->torque_setpoint = torque;
}

/**
 * This function must be called periodically to succesfully update the PID controller. The function
 * first measures the torque via T = K_t * I, where I is read from the current sensor pin. It then
 * updates the PID controller, passing the torque setpoint and measurement (ref and y) and dt. Finally,
 * the PID controller returns a control signal in volts, which must then be scaled by the actuator range
 * (in this case, 12 - 0) to obtain a value between -1 and 1. This value scales 255 to control the motor.
 */
void TorqueMotor::updateMotor(float refresh_rate) {
    this->torque_measured = this->Kt * analogRead(this->csPin);
    this->output = this->controller->calculate(this->torque_setpoint, this->torque_measured, refresh_rate);
    this->motor->setSpeed(255 * this->output / (TORQUE_PID_MAX_U - TORQUE_PID_MIN_U));
}
