#include <Arduino.h>
#include <VNHDriver.h>

void VNHDriver::begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin) {
    this->pwmPin = pwm_pin;
    this->inaPin = ina_pin;
    this->inbPin = inb_pin;
    pinMode(this->inaPin, OUTPUT);
    pinMode(this->inbPin, OUTPUT);
    analogWriteFrequency(this->pwmPin, 20000);
    this->setSpeed(0);
}


void VNHDriver::setSpeed(int speed) {

    if (speed > 0) {
        digitalWrite(this->inaPin, HIGH);
        digitalWrite(this->inbPin, LOW);
    } else if (speed < 0) {
        digitalWrite(this->inaPin, LOW);
        digitalWrite(this->inbPin, HIGH);
    } else {
        digitalWrite(this->inaPin, LOW);
        digitalWrite(this->inbPin, LOW);
    }
    int abs_speed = abs(speed);
    abs_speed = abs_speed > 255 ? 255 : abs_speed;
    analogWrite(this->pwmPin, abs_speed);
}