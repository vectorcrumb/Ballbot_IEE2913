#include "VNHDriver.h"


void VNHDriver::begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin) {
    this->pwmPin = pwm_pin;
    this->inaPin = ina_pin;
    this->inbPin = inb_pin;
    pinMode(this->inaPin, OUTPUT);
    pinMode(this->inbPin, OUTPUT);
    analogWriteFrequency(this->pwmPin, PWM_FREQ);
    this->setSpeed(0.0);
}


uint8_t VNHDriver::setSpeed(float speed) {
    // Configure direction of rotation
    if (speed > 0) {
        digitalWrite(this->inaPin, HIGH);
        digitalWrite(this->inbPin, LOW);
    } else if (speed < 0) {
        digitalWrite(this->inaPin, LOW);
        digitalWrite(this->inbPin, HIGH);
    } else {
        // In case of obtaining a 0 input, simply set to 0 DC and return
        digitalWrite(this->inaPin, LOW);
        digitalWrite(this->inbPin, LOW);
        analogWrite(this->pwmPin, 0);
        return 0;
    }
    // Set absolute value * 255 to PWM pin
    float abs_speed = fabsf(speed);
    uint8_t pwm_dc = abs_speed >= IN_LIMIT ? PWM_LIMIT : (uint8_t) (PWM_LIMIT * abs_speed);
    analogWrite(this->pwmPin, pwm_dc);
    return pwm_dc;
}