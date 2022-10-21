#include "UsvDriver.hpp"

namespace HEAR{

UsvDriver::UsvDriver(int b_uid) : Block(BLOCK_ID::GENERIC, b_uid){
    _cmd_port = createInputPort<std::vector<float>>(IP::CONTROL_CMD, "CONTROL_CMD");
    _hb_port = createInputPort<int>(IP::HB, "HB");
    right_th_cmd_port = createOutputPort<int>(OP::RIGHT_TH_CMD, "RIGHT_TH_CMD");
    left_th_cmd_port = createOutputPort<int>(OP::LEFT_TH_CMD, "LEFT_TH_CMD");
    _hb_timer.tick();
}

void UsvDriver::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        bool arm_val = ((BoolMsg*)u_msg)->data;;
        if(arm_val){
            _armed = arm_val;
            std::cout << "arm called \n";
        }
        else{
            _armed = arm_val;
            std::cout << "disarm called \n";
        }
        // print armed
    }
}

void UsvDriver::process() {
    if(_hb_enabled){
        int hb_val = 0;
        _hb_port->read(hb_val);
        if(prev_hb_val != hb_val){
            prev_hb_val = hb_val;
            _hb_timer.tick();
        }
        if ( _hb_timer.tockMilliSeconds() > _hb_tol_ms){
            _armed = false;
        }
    }
    std::vector<float> u_cmd;
    _cmd_port->read(u_cmd);
    _u[0] = u_cmd[0]; 
    _u[1] = u_cmd[1];
    if(_armed){
        this->command();
    }else{
        th_cmds[0] = _escMid;
        th_cmds[1] = _escMid;
    }
    right_th_cmd_port->write(th_cmds[0]);
    left_th_cmd_port->write(th_cmds[1]);
}

void UsvDriver::command(){

    //TODO split into more methods
    for(int i = 0; i < 2; i++){
        _commands[i] = 0.0;
    }

    // contrain inputs to be from -1 to 1
    this->constrain(_u[0], -1, 1);
    this->constrain(_u[1], -1, 1);
    
    //Update pulse values
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            _commands[i] += _geometry[i][j] * _u[j];
        }
    }

    float offset = 0;
    if(_commands[0] > 1){
        offset = _commands[0] - 1 ;
    }else if (_commands[0] < -1){
        offset = _commands[0] + 1;
    }else if(_commands[1] > 1){
        offset = _commands[1] - 1;
    }else if (_commands[1] < -1){
        offset = _commands[1] + 1;
    }

    th_cmds[0] = this->change_range(_commands[0]-offset, 1000, 1000);
    th_cmds[1] = this->change_range(_commands[1]-offset, 1000, 1000);
}

void UsvDriver::setESCValues(int t_min, int t_mid, int t_max) {
    _escMin = t_min;
    _escMid = t_mid;
    _escMax = t_max;
}

int UsvDriver::constrain(float value, int min_value, int max_value) {
    int val = static_cast<int>(value);
    if (val > max_value) {
        val = max_value;
    }
    // bug cathed in the original code
    else if (val < min_value) {
        val = min_value;
    }
    return val;
}

}