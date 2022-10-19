#ifndef __RCMANUALOVERRIDE_H__
#define __RCMANUALOVERRIDE_H__

#include "HEAR_core/Block.hpp"

namespace HEAR{

class RCManualOverride :public Block {
private:
    OutputPort<std::vector<float>> *control_cmd_out_;
    InputPort<std::vector<float>> *rc_cmd_in_, *control_cmd_in_;
    InputPort<std::vector<int>> *rc_raw_in_;
    uint8_t rc_override_chan = 6;
    std::vector<int> rc_raw;
    std::vector<float> in_cmd_rc, in_cmd_cont;

    int last_val = 0;
    bool rc_override_ = false;
    bool initialized_ = false;

public:
    enum OP{CMD_OUT};
    enum IP{RC_RAW, RC_CMD, CONTROL_CMD};
    RCManualOverride(int b_uid);
    ~RCManualOverride(){}
    void process();
    void setRCoverrideChan(uint8_t chan_idx){
        rc_override_chan = chan_idx;
    }
    void reset() override{
        initialized_ = false;
        last_val = 0;
        rc_override_ = false;
    }

};

}

#endif // __RCMANUALOVERRIDE_H__