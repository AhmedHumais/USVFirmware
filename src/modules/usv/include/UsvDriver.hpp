#ifndef USVDRIVER_HPP
#define USVDRIVER_HPP

#include <vector>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Timer.hpp"

namespace HEAR{

class UsvDriver : public Block {
protected: 
    Timer _hb_timer;
    int _hb_tol_ms = 1000;
    float _u[2]; //[fwd, yaw]
    int prev_hb_val = 0;
    std::vector<int> th_cmds {1500, 1500};

    std::vector<float> _commands {0,0};
    float _geometry[2][2] = {{-1,  1},
                             {1,   1}};


    InputPort<std::vector<float>>* _cmd_port;
    InputPort<int>* _hb_port;
    OutputPort<int>* right_th_cmd_port;
    OutputPort<int>* left_th_cmd_port;

    int _escMin = 1000;
    int _escMid = 1500;
    int _escMax = 2000;
    
    int constrain(float value, int min_value, int max_value);
    void command();

public:
    enum IP{CONTROL_CMD, HB};
    enum OP{RIGHT_TH_CMD, LEFT_TH_CMD};
    void process();
    bool _hb_enabled = true;
    bool _armed = false;
    void setHbTol(int hb_tol_ms){
        _hb_tol_ms = hb_tol_ms;
    }
    int change_range(float val, float multiplier, int offset){
        return val*multiplier+offset;
    }
    void constrain(float &val,const float &min, const float &max) {
        if (val > max) { val = max;}
        else if (val < min) { val = min;}
    }
    void update(UpdateMsg* u_msg) override;
    void setESCValues(int, int, int);
    UsvDriver(int b_uid);
    ~UsvDriver(){}
}; 

}

#endif