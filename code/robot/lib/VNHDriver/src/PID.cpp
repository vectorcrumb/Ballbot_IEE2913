#include "PID.h"

class PIDImpl {
public:
    PIDImpl(float Kp, float Ki, float Kd, float sat_min, float sat_max, float dt);
    ~PIDImpl();
    float calculate(float ref, float y);
    float calculate(float ref, float y, float dt);
private:
    float _dt;
    float _max, _min;
    float _Kp, _Ki, _Kd;
    float _prev_err, _int_err;
};

PID::PID(float Kp, float Ki, float Kd, float sat_min, float sat_max, float dt) {
    pimpl = new PIDImpl(Kp, Ki, Kd, sat_min, sat_max, dt);
}

float PID::calculate(float ref, float y) {
    return pimpl->calculate(ref, y);
}

float PID::calculate(float ref, float y, float dt) {
    return pimpl->calculate(ref, y, dt);
}

PID::~PID() {
    delete pimpl;
}

PIDImpl::PIDImpl(float Kp, float Ki, float Kd, float sat_min, float sat_max, float dt) :
    _dt(dt),
    _max(sat_max),
    _min(sat_min),
    _Kp(Kp),
    _Ki(Ki),
    _Kd(Kd),
    _prev_err(0),
    _int_err(0) {

}

float PIDImpl::calculate(float ref, float y) {
    return PIDImpl::calculate(ref, y, _dt);
}

float PIDImpl::calculate(float ref, float y, float dt) {
    float err = ref - y;
    _int_err += err * dt;

    float pOut = _Kp * err;
    float iOut = _Ki * _int_err;
    float dOut = _Kd * (err - _prev_err) / dt;
    
    float u = pOut + iOut + dOut;
    if (u > _max) u = _max;
    else if (u < _min) u = _min;

    _prev_err = err;
    
    return u;
}

PIDImpl::~PIDImpl() {

}
