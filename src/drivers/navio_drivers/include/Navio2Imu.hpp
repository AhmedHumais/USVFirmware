#ifndef NAVIO2IMU_HPP
#define NAVIO2IMU_HPP

#include "HEAR_core/Block.hpp"
#include "Common/MPU9250.h"
#include "Common/Util.h"
#include <memory>
#include <vector>
#include "Filters/MadgwickFilter.hpp"
#include "Filters/MahonyFilter.hpp"
#include "HEAR_core/Vector3D.hpp"

#include <iostream>

namespace HEAR{

class Navio2Imu : public Block{
private:
    OutputPort<Vector3D<float>>* rpy_port;
    OutputPort<Vector3D<float>>* gyro_port;
    OutputPort<Vector3D<float>>* acc_port;
    std::unique_ptr <InertialSensor> sensor;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    HEAR_filter::Filter* filter;
    float _fs;
    bool sensor_available = false;
    float gyroOffset[3] = {0, 0, 0};
    float accOffset[3] = {0, 0, 0};
    void removeOffset(){
        ax -= accOffset[0];
        ay -= accOffset[1];
        az -= accOffset[2];

        gx  -= gyroOffset[0];
        gy  -= gyroOffset[1];
        gz  -= gyroOffset[2];
    }

public:
    enum OP{RPY, GYRO, ACC};
    Navio2Imu(int b_uid, float freq);
    ~Navio2Imu(){}
    void process();
    void calibrateIMU();

    void setGyroOffset(const float &x, const float &y, const float &z){
        gyroOffset[0] = x;
        gyroOffset[1] = y;
        gyroOffset[2] = z;
    }
    void setAccOffset(const float &x, const float &y, const float &z){
        accOffset[0] = x;
        accOffset[1] = y;
        accOffset[2] = z;
    }

    Vector3D<float> ToEulerAngles(Quaternion q);

};

}

#endif