
#ifndef MADGWICKFILTER_HPP
#define MADGWICKFILTER_HPP

#include "Filters/Filter.hpp"
#include <math.h>

namespace HEAR_filter{

class MadgwickFilter: public Filter{

private:
    float beta;								// 2 * proportional gain (Kp)
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
public:
    void update();
    MadgwickFilter(float fs, float beta = 0.1): Filter(fs){
    }
    void MadgwickUpdate();
    void MadgwickUpdateIMU();
};
}
#endif
