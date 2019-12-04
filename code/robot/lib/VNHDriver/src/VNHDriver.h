#ifndef VNHDRIVER_h
#define VNHDRIVER_h

#include <inttypes.h>

class VNHDriver {
public:
    void begin(uint8_t pwm_pin, uint8_t ina_pin, uint8_t inb_pin);
    void setSpeed(int speed);
    int speed;
    int motor_type;
private:
    uint8_t pwmPin, inaPin, inbPin;
};

#endif