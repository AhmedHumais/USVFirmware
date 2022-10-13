#ifndef RADIOCONTROLLER_HPP
#define RADIOCONTROLLER_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"
#include <vector>
#include <iostream>

namespace HEAR{

enum CHANNEL_NAME{
    THRUST = 0, 
    ROLL = 1,
    PITCH = 2,
    YAW = 3
};

class RadioController : public Block{
private:
    InputPort<std::vector<float>>* rc_command_port;
    OutputPort<float>* thrust_cmd_port;
    OutputPort<Vector3D<float>>* torque_cmd_port;
    std::vector<uint8_t> channel_map = {2, 0, 1, 3}; 
    std::vector<float> rc_in = {0, 0, 0, 0};
    const int MAX_CHAN = 14;
    float r_min = 1150, r_max= 2000, r_mid = 1500;
    float p_min = 1150, p_max= 2000, p_mid = 1500;
    float y_min = 1150, y_max= 2000, y_mid = 1500;
    float t_min = 1150, t_max= 2000;

    float r_cmd = 0, p_cmd = 0, y_cmd = 0, t_cmd = 0;
    bool start_calib = false, stop_calib = false, calibrate_rc = false;
    void map_rc();
    void calibrateRC();

public:
    enum IP{RC_COMMAND};
    enum OP{THRUST_CMD, TORQUES_CMD};
    RadioController(int b_uid);
    ~RadioController(){}
    void process();
    void update(UpdateMsg* u_msg) override;

};

}

#endif