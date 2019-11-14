#ifndef _VNHDRIVER_h
#define _VNHDRIVER_h

#include <Arduino.h>
#include <inttypes.h>


#define MOTOR_PWM_FREQUENCY 20000


class VNHDriver {
public:
    void begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin);
    void setSpeed(int speed);
private:
    void initialState();
    uint8_t pwmPin, inaPin, inbPin;
};


#endif