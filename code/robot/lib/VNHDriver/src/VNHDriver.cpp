#include <Arduino.h>
#include <VNHDriver.h>

void VNHDriver::begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin, uint8_t motor_type) {
    if (motor_type==1){
        this->pwmPin = pwm_pin;
        this->inaPin = ina_pin;
        this->inbPin = inb_pin;
        pinMode(this->inaPin, OUTPUT);
        pinMode(this->inbPin, OUTPUT);
        analogWriteFrequency(this->pwmPin, 20000);
        this->setSpeed(0, 1);
    }
    else if (motor_type==2){
        this->pwmPin = pwm_pin;
        this->inaPin = ina_pin;
        this->inbPin = inb_pin;
        pinMode(this->inbPin, OUTPUT);
        analogWriteFrequency(this->pwmPin, 20000);
        analogWriteFrequency(this->inaPin, 20000);
        this->setSpeed(0, 2);
    }
}

//Driver 2
//pwmPin -> LPWM
//inbPin -> EN
//inaPin -> RPWM

void VNHDriver::setSpeed(int speed, uint8_t motor_type) {
    if (motor_type==1){
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
    if (motor_type==2){
        int abs_speed = abs(speed);
        abs_speed = abs_speed > 255 ? 255 : abs_speed;
        if (speed<0){
            digitalWrite(this->inbPin, HIGH);
            analogWrite(this->inaPin, 0);
            analogWrite(this->pwmPin, abs_speed);
        } else if (speed>0){
            digitalWrite(this->inbPin, HIGH);
            analogWrite(this->pwmPin, 0);
            analogWrite(this->inaPin, abs_speed);
        } else {
            digitalWrite(this->inbPin, LOW);
            analogWrite(this->pwmPin, 0);
            analogWrite(this->inaPin, 0);
        }
    }
}