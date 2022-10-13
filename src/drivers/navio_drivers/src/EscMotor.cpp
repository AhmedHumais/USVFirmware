#include "EscMotor.hpp"

namespace HEAR{

ESCMotor::ESCMotor(int t_pin, int t_freq){
    _pwmPin = t_pin;
    _freq = t_freq;
    this->initialize();
}

bool ESCMotor::initialize(){

    _pwm = new RCOutput_Navio2();

    if(!(_pwm->initialize(_pwmPin)) ) {
        return 1;
    }

    _pwm->set_frequency(_pwmPin, _freq);

	if ( !(_pwm->enable(_pwmPin)) ) {
	    return 1;
	}

}

void ESCMotor::applyCommand(int t_command){

    //std::cout << "Received Command on PIN: " << _pwmPin << " Value :" << t_command << "\r"; //std::endl;
    _pwm->set_duty_cycle(_pwmPin, t_command);

}

}
