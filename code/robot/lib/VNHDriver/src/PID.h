#ifndef _PID_H_
#define _PID_H_


class PIDImpl;

class PID {
public:
    PID(float Kp, float Ki, float Kd, float sat_min, float sat_max, float dt);
    float calculate(float ref, float y);
    float calculate(float ref, float y, float dt);
    ~PID();
private:
    PIDImpl *pimpl;
};

#endif