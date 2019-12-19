#ifndef _MOV_AVG_H
#define _MOV_AVG_H

#include <inttypes.h>


#define FILTER_ORDER 50

class MovingAverageFilter {
public:
    MovingAverageFilter();
    ~MovingAverageFilter();
    float updateFilter(float measurement);
    float peekFilter();
private:
    float samples[FILTER_ORDER] = { };
    uint16_t idx;
};

#endif