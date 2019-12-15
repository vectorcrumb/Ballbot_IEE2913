#ifndef VNHDRIVER_h
#define VNHDRIVER_h

#include <inttypes.h>
#include "Arduino.h"


#define PWM_FREQ 10000
#define IN_LIMIT 1.0
#define PWM_LIMIT 255

class VNHDriver {
public:
    void begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin);
    uint8_t setSpeed(float speed);
private:
    uint8_t pwmPin, inaPin, inbPin;
};

#endif