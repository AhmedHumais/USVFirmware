#ifndef RCIN_HPP
#define RCIN_HPP

#include "HEAR_core/Block.hpp"
#include "Navio2/RCInput_Navio2.h"
#include "Common/Util.h"
#include <memory>
#include <vector>

namespace HEAR{

class RCin : public Block{
private:
    OutputPort<std::vector<int>>* rc_command_port;
    std::unique_ptr <RCInput> rc_in;
    int NUM_CHANNELS = 4;
    const int MAX_CHAN = 14;
public:
    enum OP{RC_COMMAND};
    RCin(int b_uid);
    RCin(int b_uid, size_t num_channels);
    ~RCin(){}
    void process();

};

}

#endif