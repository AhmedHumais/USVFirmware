#include "UsvRadioController.hpp"

namespace HEAR{

UsvRadioController::UsvRadioController(int b_uid) : Block(BLOCK_ID::RADIO_CONTROLLER, b_uid){
    rc_command_port = createInputPort<std::vector<int>>(IP::RC_COMMAND, "RC_COMMAND");
    cmd_port = createOutputPort<std::vector<float>>(OP::CONTROL_CMD, "CONTROL_CMD");
}

void UsvRadioController::process(){
    rc_command_port->read(rc_in);
    if(stop_calib){
        if( (f_max <= f_mid+100) || (y_max <= y_mid+100)){
            std::cout << "Calibration unsuccessful"<<std::endl;
        } else {
            std::cout << "Calibration Successful" <<std::endl;
            this->saveCalib();
        }
        std::cout << "Forward trim: " << f_min << ", " << f_mid << ", " << f_max <<std::endl;        
        std::cout << "Yaw trim: " << y_min << ", " << y_mid << ", " << y_max <<std::endl;        
        stop_calib = false;
    }
    if(calibrate_rc){
        if(start_calib){
            std::cout << "starting Calibration" <<std::endl;
            f_mid = rc_in[channel_map[CHANNEL_NAME::FWD]];
            y_mid = rc_in[channel_map[CHANNEL_NAME::YAW]];
            f_max = f_mid + 100; f_min = f_mid - 100;
            y_max = y_mid + 100; y_min = f_mid - 100;
            start_calib = false;
        }
        this->calibrateRC();
    }
    else{
        cmd_port->write(this->map_rc());
    }
}

std::vector<float> UsvRadioController::map_rc(){
    std::vector<float> _cmd(2, 0);
    float f = rc_in[channel_map[CHANNEL_NAME::FWD]];
    float y = rc_in[channel_map[CHANNEL_NAME::YAW]];
    if(f == 0 || y == 0){
        std::cout << "Invalid RC commands" << std::endl;
        return _cmd;
    }

    _cmd[0] = f - _calib_params.mid[0];
    if((_cmd[0] >= 0)){
        _cmd[0] = _cmd[0]/(_calib_params.max[0]-_calib_params.mid[0]);
    } else{ 
        _cmd[0] = _cmd[0]/(_calib_params.mid[0]-_calib_params.min[0]);
    }

    _cmd[1] = y - _calib_params.mid[1];
    if((_cmd[1] >= 0)){
        _cmd[1] = _cmd[1]/(_calib_params.max[1]-_calib_params.mid[1]);
    } else{ 
        _cmd[1] = _cmd[1]/(_calib_params.mid[1]-_calib_params.min[1]);
    }


    this->constrain(_cmd[0], -1, 1);
    this->constrain(_cmd[1], -1, 1);

    if(reverse_fwd){
        _cmd[0] = (-1)*_cmd[0];
    }
    if(reverse_yaw){
        _cmd[1] = (-1)*_cmd[1];
    }

    // _cmd[0] = change_range(_cmd[0], 1000, 1000);
    // _cmd[1] = change_range(_cmd[1], 1000, 1000);

    return _cmd;
}

void UsvRadioController::calibrateRC(){
    
    rc_in[channel_map[CHANNEL_NAME::FWD]] > f_max? f_max = rc_in[channel_map[CHANNEL_NAME::FWD]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::FWD]] < f_min? f_min = rc_in[channel_map[CHANNEL_NAME::FWD]] : 0;

    rc_in[channel_map[CHANNEL_NAME::YAW]] > y_max? y_max = rc_in[channel_map[CHANNEL_NAME::YAW]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::YAW]] < y_min? y_min = rc_in[channel_map[CHANNEL_NAME::YAW]] : 0;
        
}

void UsvRadioController::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        calibrate_rc = ((BoolMsg*)u_msg)->data;
        if(calibrate_rc) {
            start_calib = true;
            stop_calib = false;
        }
        else{
            stop_calib = true;
        }
    }
}
}