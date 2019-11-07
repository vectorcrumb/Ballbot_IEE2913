#ifndef _UTIL_H_
#define _UTIL_H_

#include <Arduino.h>

float clamp_value(float value, float min_val, float max_val);
float clamp_value(float value, float limit);
float newton_derivative(float f, float f_prev, float h);

#endif