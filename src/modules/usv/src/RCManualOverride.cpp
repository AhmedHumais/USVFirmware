#include "RCManualOverride.hpp"

namespace HEAR {

RCManualOverride::RCManualOverride(int b_uid) : Block(BLOCK_ID::TEMPLATE, b_uid){
    control_cmd_out_ = createOutputPort<std::vector<float>>(OP::CMD_OUT, "CMD_OUT");
    rc_cmd_in_ = createInputPort<std::vector<float>>(IP::RC_CMD, "RC_CMD");
    rc_raw_in_ = createInputPort<std::vector<int>>(IP::RC_RAW, "RC_RAW");
    control_cmd_in_ = createInputPort<std::vector<float>>(IP::CONTROL_CMD, "CONTROL_CMD");

    rc_raw = std::vector<int>(14, 0);
    in_cmd_cont = std::vector<float>(2, 0);
    in_cmd_rc = std::vector<float>(2, 0);
}

void RCManualOverride::process(){
    rc_raw_in_->read(rc_raw);
    int rc_val;
    if(rc_raw.size() <= rc_override_chan){
        std::cout << "[ERROR] RC override block not receiving data properly." << std::endl;
    }
    else{
        rc_val = rc_raw[rc_override_chan];
        if(!initialized_){
            if(rc_val != 0){
                last_val = rc_val;
                initialized_ = true;
            }
            else{
                std::cout<< "[WARN] RC override: No data on the rc_override_channel." << std::endl;
            }
        }
        else{
            if(rc_val != last_val){
                rc_override_ = true;
            }
        }
    }
    if(rc_override_){
        std::cout <<"RC Override active" <<std::endl;
        rc_cmd_in_->read(in_cmd_rc);
        control_cmd_out_->write(in_cmd_rc);
    }
    else{
        control_cmd_in_->read(in_cmd_cont);
        control_cmd_out_->write(in_cmd_cont);
    }
}


}