#ifndef RADIOCONTROLLER_HPP
#define RADIOCONTROLLER_HPP

#include "HEAR_core/Block.hpp"
#include "usv_utils.hpp"
#include <vector>
#include <iostream>

namespace HEAR{

enum CHANNEL_NAME{
    FWD = 0, 
    YAW = 1,
};

struct UsvRCcalibParams{
    int mid[2] = {1500, 1500};
    int min[2] = {1000, 1000};
    int max[2] = {2000, 2000};
};

class UsvRadioController : public Block{
private:
    InputPort<std::vector<int>>* rc_command_port;
    OutputPort<std::vector<float>>* cmd_port;
    std::vector<uint8_t> channel_map = {0, 1};
    std::vector<int> rc_in = {0, 0};
    UsvRCcalibParams _calib_params;
    const int MAX_CHAN = 14;
    int RC_MAX = 2000, RC_MIN=1000, RC_MID=1500;
    bool start_calib = false, stop_calib = false, calibrate_rc = false;
    std::vector<float> map_rc();
    void calibrateRC();

    float f_min = 1150, f_max= 2000, f_mid = 1500;
    float y_min = 1150, y_max= 2000, y_mid = 1500;

public:
    enum IP{RC_COMMAND};
    enum OP{THRUST_CMD, TORQUES_CMD};
    void setChanMap(uint8_t fwd_chan, uint8_t yaw_chan){
        if(fwd_chan> MAX_CHAN || yaw_chan > MAX_CHAN){
            return;
        }
        channel_map[FWD] = fwd_chan;
        channel_map[YAW] = yaw_chan;
    }
    void setCalibParams(const UsvRCcalibParams& params){
        _calib_params = params;
    }
    void saveCalib(){
        _calib_params.max[0] = f_max; _calib_params.max[1] = y_max;
        _calib_params.mid[0] = f_mid; _calib_params.mid[1] = y_mid;
        _calib_params.min[0] = f_min; _calib_params.min[1] = y_min;
    }
    void constrain(float &val,const float &min, const float &max) {
        if (val > max) {
            val = max;
        }
        else if (val < min) {
            val = min;
        }
    }
    // int change_range(int val, int multiplier, int offset){
    //     return val*multiplier+offset;
    // }
    UsvRadioController(int b_uid);
    ~UsvRadioController(){}
    void process();
    void update(UpdateMsg* u_msg) override;

};

}

#endif