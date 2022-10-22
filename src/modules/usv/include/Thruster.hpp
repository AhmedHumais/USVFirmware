#ifndef THRUSTER_HPP
#define THRUSTER_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Timer.hpp"
#include "EscMotor.hpp"

#include "usv_utils.hpp"

namespace HEAR{

class Thruster :public Block {
private:
    OutputPort<int>* _out;
    InputPort<int>* _in;
    ESCMotor* _motor;
    int THRUSTER_MAX = 2000, THRUSTER_MIN = 1000;
    int _init_val = 1500;
    bool initialized = false;
    Timer _timer;
    float init_time_ms = 2000, _freq = 400;
    bool enable = true;   
public:
    enum OP{OUT};
    enum IP{INP};
    Thruster(int b_uid, int idx);
    Thruster(int b_uid, size_t idx, int freq, int init_val);
    ~Thruster(){}
    int constrain(int);
    void init();
    void process();
    void update(UpdateMsg* u_msg) override;
    void reset() override {
        initialized = false;
        this->init();
    }

};

}

#endif