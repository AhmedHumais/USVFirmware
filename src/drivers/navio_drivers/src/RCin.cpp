#include "RCin.hpp"

namespace HEAR{

RCin::RCin(int b_uid) : Block(BLOCK_ID::RC_IN, b_uid){
    rc_command_port = createOutputPort<std::vector<int>>(OP::RC_COMMAND, "RC_COMMAND");
    rc_in = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    rc_in->initialize();
}

RCin::RCin(int b_uid, size_t num_channels) : RCin(b_uid){
    NUM_CHANNELS = num_channels;
}

void RCin::process(){
    std::vector<int> x(NUM_CHANNELS, 0);
    for (int i=0; i<NUM_CHANNELS; i++){
        x[i] = rc_in->read(i); 
    }
    rc_command_port->write(x);
}


}