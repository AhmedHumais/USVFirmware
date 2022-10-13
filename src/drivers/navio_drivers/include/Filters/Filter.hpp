#ifndef FILTER_HPP
#define FILTER_HPP
#include <vector>

namespace HEAR{

struct Quaternion {
    double w, x, y, z;
};

}

namespace HEAR_filter{

class Filter{
protected:
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float sampleFreq;
    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
public:
    HEAR::Quaternion quat;
    Filter(const float &fs) : sampleFreq(fs){
        ax=0; ay=0; az=0;
        gx=0; gy=0; gz=0;
        mx=0; my=0; mz=0;
    }
    virtual ~Filter(){}

    virtual void update() = 0;
    void input(const float &gx_, const float &gy_, const float &gz_, const float &ax_, const float &ay_, const float &az_, const float &mx_, const float &my_, const float &mz_){
        ax = ax_; ay = ay_; az = az_;
        gx = gx_; gy = gy_; gz = gz_;
        mx = mx_; my = my_; mz = mz_;
    }
    void input(const float &gx_, const float &gy_, const float &gz_, const float &ax_, const float &ay_, const float &az_){
        ax = ax_; ay = ay_; az = az_;
        gx = gx_; gy = gy_; gz = gz_;
    }
};

}

#endif