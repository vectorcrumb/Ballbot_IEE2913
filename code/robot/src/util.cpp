#include "util.h"

float clamp_value(float value, float min_val, float max_val) {
    if (value > max_val) {
        return max_val;
    } else if (value < min_val) {
        return min_val;
    }
    return value;
}

float clamp_value(float value, float limit) {
    if (abs(value) > limit) {
        return limit * (value > 0 ? 1 : -1);
    }
    return value;
}

float newton_derivative(float f, float f_prev, float h) {
    return (f - f_prev) / h;
}