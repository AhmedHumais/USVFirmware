#include "Thruster.hpp"

namespace HEAR {

Thruster::Thruster(int b_uid, int idx) : Block(BLOCK_ID::GENERIC, b_uid){
    _in = createInputPort<int>(IP::INP, "Input");
    _motor = new ESCMotor(idx, _freq);
    this->init();
}

Thruster::Thruster(int b_uid, size_t idx, int freq, int init_val) : Thruster(b_uid, idx){
    _freq = freq;
    _init_val = init_val;
}

void Thruster::process(){
    if(initialized){
        if(enable){
            int x;
            _in->read(x);
            _motor->applyCommand(constrain(x));
        }
        else{
            _motor->applyCommand(_init_val);
        }
    }
    else if(_timer.tockMilliSeconds() > init_time_ms){
        initialized = true;
    }
}

void Thruster::init(){
    _motor->applyCommand(_init_val);
    _timer.tick();
}

int Thruster::constrain(int val) {
    if (val > THRUSTER_MAX) {
        val = THRUSTER_MAX;
    }
    else if (val < THRUSTER_MIN) {
        val = THRUSTER_MIN;
    }
    return val;
}
void Thruster::update(UpdateMsg* u_msg){
    if(u_msg->getType() == USVMSG){
        if(((UsvMsg*)u_msg)->getUsvMsgType() == THRUSTER_RESET){
            initialized = false;
            this->init();
        }
        if(((UsvMsg*)u_msg)->getUsvMsgType() == THRUSTER_ENABLE){
            this->enable = ((ThrusterMsg*)u_msg)->enable;
        }
    }
}

}