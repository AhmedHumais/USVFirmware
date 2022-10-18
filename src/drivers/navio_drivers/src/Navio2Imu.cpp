#include "Navio2Imu.hpp"

#define G_SI 9.80665

namespace HEAR{

Navio2Imu::Navio2Imu(int b_uid, float freq) : Block(BLOCK_ID::NAVIO2IMU, b_uid){
    _fs = freq;
    rpy_port = createOutputPort<Vector3D<float>>(OP::RPY, "QUAT");
    gyro_port = createOutputPort<Vector3D<float>>(OP::GYRO, "GYRO");
    acc_port = createOutputPort<Vector3D<float>>(OP::ACC, "ACC");
    sensor = std::unique_ptr <InertialSensor>{ new MPU9250() };
    filter = new HEAR_filter::MahonyFilter(_fs);
    if(sensor->probe()){
        sensor_available = true;
        sensor->initialize();
    }
}

void Navio2Imu::process(){
    if(sensor_available){
        sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        // sensor->read_magnetometer(&mx, &my, &mz);
        
        removeOffset();
        filter->input(gy, -gx, gz, ay, -ax, az);
        filter->update();
        Vector3D<float> ang_rates(gy, -gx, gz);
        Vector3D<float> acceleration(ay, -ax, az);

        rpy_port->write(this->ToEulerAngles(filter->quat));
        gyro_port->write(ang_rates);
        acc_port->write(acceleration);
        
    }
}

void Navio2Imu::calibrateIMU(){
    if(sensor_available){
        std::cout <<"Calibrating IMU ... " << std::endl;
        float sum_gyro[3] = {0, 0, 0};
        float sum_acc[3] = {0, 0, 0};
        float dt = 1.0/_fs;
        for (int i=0; i < 5000; i++){
            sensor->update();
            sensor->read_accelerometer(&ax, &ay, &az);
            sensor->read_gyroscope(&gx, &gy, &gz);
            
            sum_acc[0] += ax*dt; sum_acc[1] += ay*dt; sum_acc[2] += az*dt;
            sum_gyro[0] += gx*dt; sum_gyro[1] += gy*dt; sum_gyro[2] += gz*dt;
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
        float sc = 5000*dt;
        float bias_gx = sum_gyro[0]/sc;
        float bias_gy = sum_gyro[1]/sc;
        float bias_gz = sum_gyro[2]/sc;
        
        float bias_ax = sum_acc[0]/sc;
        float bias_ay = sum_acc[1]/sc;
        float bias_az = sum_acc[2]/sc - G_SI;
        setGyroOffset(bias_gx, bias_gy, bias_gz);
        setAccOffset(bias_ax, bias_ay, bias_az);

        std::cout << "Gyro biases: "<< bias_gx << ", " << bias_gy << ", " << bias_gz <<std::endl;
        std::cout << "Acc biases: "<< bias_ax << ", " << bias_ay << ", " << bias_az << std::endl;
    }
}

Vector3D<float> Navio2Imu::ToEulerAngles(Quaternion q) {
    Vector3D<float> angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
}