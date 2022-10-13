
#ifndef MAHONYFILTER_HPP
#define MAHONYFILTER_HPP

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.001f)	// 2 * integral gain

#include "Filters/Filter.hpp"
#include <math.h>

namespace HEAR_filter{

class MahonyFilter: public Filter{

private:
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
public:
    void update();
    MahonyFilter(float fs): Filter(fs){
    }
    void MahonyUpdate();
    void MahonyUpdateIMU();
};
}
#endif
