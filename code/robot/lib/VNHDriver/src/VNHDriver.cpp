#include "VNHDriver.h"

void VNHDriver::begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin) {
    this->pwmPin = pwm_pin;
    this->inaPin = ina_pin;
    this->inbPin = inb_pin;
    pinMode(this->inaPin, OUTPUT);
    pinMode(this->inbPin, OUTPUT);
    analogWriteFrequency(this->pwmPin, MOTOR_PWM_FREQUENCY);
    this->initialState();
}

void VNHDriver::setSpeed(int speed) {
    if (speed > 0) {
        digitalWriteFast(this->inaPin, HIGH);
        digitalWriteFast(this->inbPin, LOW);
    } else if (speed < 0) {
        digitalWriteFast(this->inaPin, LOW);
        digitalWriteFast(this->inbPin, HIGH);
    } else {
        digitalWriteFast(this->inaPin, LOW);
        digitalWriteFast(this->inbPin, LOW);
    }
    int abs_speed = abs(speed);
    abs_speed = abs_speed > 255 ? 255 : abs_speed;
    analogWrite(this->pwmPin, abs_speed);
}

void VNHDriver::initialState() {
    digitalWriteFast(this->inaPin, LOW);
    digitalWriteFast(this->inbPin, LOW);
    digitalWriteFast(this->pwmPin, LOW);
}
