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
    float delta_time = dt / 1000;
    _int_err += err * delta_time;

    // Serial.print(F("PID err: ")); Serial.println(err);
    // Serial.print(F("PID int_err: ")); Serial.println(_int_err);

    float iOut = _Ki * err;
    //float iOut = _Ki * _int_err;
    float pOut = _Kp * (err - _prev_err) / delta_time;

    // Serial.print(F("dt: ")); Serial.println(dt);
    // Serial.print(F("Ref: ")); Serial.println(ref);

    // Serial.print(F("PID pOut: ")); Serial.println(pOut);
    // Serial.print(F("PID iOut: ")); Serial.println(iOut);
    // Serial.print(F("PID dOut: ")); Serial.println(dOut);

    float u = pOut + iOut;
    if (u > _max) u = _max;
    else if (u < _min) u = _min;

    _prev_err = err;

    // Serial.print(F("PID out: ")); Serial.println(u);

    return u;
}

PIDImpl::~PIDImpl() {

}