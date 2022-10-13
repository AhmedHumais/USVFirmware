#include "RadioController.hpp"

namespace HEAR{

RadioController::RadioController(int b_uid) : Block(BLOCK_ID::RADIO_CONTROLLER, b_uid){
    rc_command_port = createInputPort<std::vector<float>>(IP::RC_COMMAND, "RC_COMMAND");
    thrust_cmd_port = createOutputPort<float>(OP::THRUST_CMD, "THRUST_CMD");
    torque_cmd_port = createOutputPort<Vector3D<float>>(OP::TORQUES_CMD, "TORQUES_CMD");
}

void RadioController::process(){
    rc_command_port->read(rc_in);
    if(calibrate_rc){
        this->calibrateRC();
    }
    if(stop_calib){
        if( (r_max <= r_mid+100) || (p_max <= p_mid+100) || (y_max <= y_mid+100) || (t_max <= r_mid+100)){
            std::cout << "Calibration unsuccessful"<<std::endl;
        } else {
            std::cout << "Calibration Successful" <<std::endl;
        }

        std::cout << "Thrust trim: " << t_min << ", " << t_max <<std::endl;        
        std::cout << "Roll trim: " << r_min << ", " << r_mid << ", " << r_max <<std::endl;        
        std::cout << "Pitch trim: " << p_min << ", " << p_mid << ", " << p_max <<std::endl;        
        std::cout << "Yaw trim: " << y_min << ", " << y_mid << ", " << y_max <<std::endl;        
        stop_calib = false;
    }

    this->map_rc();
    Vector3D<float> torque_cmd(r_cmd, p_cmd, y_cmd);
    thrust_cmd_port->write(t_cmd);
    torque_cmd_port->write(torque_cmd);
}

void RadioController::map_rc(){
    float r = rc_in[channel_map[CHANNEL_NAME::ROLL]];
    float p = rc_in[channel_map[CHANNEL_NAME::PITCH]];
    float y = rc_in[channel_map[CHANNEL_NAME::YAW]];
    float t = rc_in[channel_map[CHANNEL_NAME::THRUST]];

    t_cmd = (t-t_min)/(t_max - t_min);
    
    p_cmd = p - p_mid;
    p_cmd = (p_cmd >= 0)? p_cmd/(p_max-p_mid) : p_cmd/(p_mid-p_min);

    r_cmd = r - r_mid;
    r_cmd = (r_cmd >= 0)? r_cmd/(r_max-r_mid) : r_cmd/(r_mid-r_min);

    y_cmd = y - y_mid;
    y_cmd = (y_cmd >= 0)? y_cmd/(y_max-y_mid) : y_cmd/(y_mid-y_min);
    
}

void RadioController::calibrateRC(){
    if(start_calib){
        std::cout << "starting Calibration" <<std::endl;
        r_mid = rc_in[channel_map[CHANNEL_NAME::ROLL]];
        p_mid = rc_in[channel_map[CHANNEL_NAME::PITCH]];
        y_mid = rc_in[channel_map[CHANNEL_NAME::YAW]];
        r_max = r_mid + 100; r_min = r_mid - 100;
        p_max = p_mid + 100; p_min = r_mid - 100;
        y_max = y_mid + 100; y_min = r_mid - 100;
        t_max = r_mid+100;
        t_min = r_mid; 
        start_calib = false;
    }
    rc_in[channel_map[CHANNEL_NAME::ROLL]] > r_max? r_max = rc_in[channel_map[CHANNEL_NAME::ROLL]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::ROLL]] < r_min? r_min = rc_in[channel_map[CHANNEL_NAME::ROLL]] : 0;

    rc_in[channel_map[CHANNEL_NAME::PITCH]] > p_max? p_max = rc_in[channel_map[CHANNEL_NAME::PITCH]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::PITCH]] < p_min? p_min = rc_in[channel_map[CHANNEL_NAME::PITCH]] : 0;
    
    rc_in[channel_map[CHANNEL_NAME::YAW]] > y_max? y_max = rc_in[channel_map[CHANNEL_NAME::YAW]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::YAW]] < y_min? y_min = rc_in[channel_map[CHANNEL_NAME::YAW]] : 0;
    
    rc_in[channel_map[CHANNEL_NAME::THRUST]] > t_max? t_max = rc_in[channel_map[CHANNEL_NAME::THRUST]] : 0; 
    rc_in[channel_map[CHANNEL_NAME::THRUST]] < t_min? t_min = rc_in[channel_map[CHANNEL_NAME::THRUST]] : 0;
    
}

void RadioController::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        calibrate_rc = ((BoolMsg*)u_msg)->data;
        if(calibrate_rc) {
            start_calib = true;
        }
        else{
            stop_calib = true;
        }

    }
}
}