#include "movavg.h"

MovingAverageFilter::MovingAverageFilter() {
    this->idx = 0;
};

MovingAverageFilter::~MovingAverageFilter() {
}

float MovingAverageFilter::updateFilter(float measurement) {
    this->samples[this->idx] = measurement;
    this->idx = (this->idx + 1) % FILTER_ORDER;
    return this->peekFilter();
}

float MovingAverageFilter::peekFilter() {
    float temp_sum = 0;
    for (int i = 0; i < FILTER_ORDER; i++) {
        temp_sum += this->samples[i];
    }
    return temp_sum / (float) FILTER_ORDER;
}