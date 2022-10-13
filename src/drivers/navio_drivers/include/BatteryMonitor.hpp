#ifndef BATTERYMONITOR_HPP
#define BATTERYMONITOR_HPP

#include "HEAR_core/Block.hpp"
#include "Navio2/ADC_Navio2.h"
#include <memory>

namespace HEAR {

class BatteryMonitor : public Block {
private:
    OutputPort<float>* bat_volt_port;
    std::unique_ptr <ADC> adc;
    const int CHANNEL = 2;
    float OFFSET = 0;
    float SCALE = 1;
    int max_chan = 0;
public:
    enum OP{BAT_VOLT};
    BatteryMonitor(int b_uid);
    void setScaleOffset(float scale, float offset){
        SCALE = scale;
        OFFSET = offset;
    }
    ~BatteryMonitor(){}
    void process();

};

}

#endif