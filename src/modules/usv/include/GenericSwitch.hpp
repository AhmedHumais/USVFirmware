#ifndef __GENERICSWITCH_H__
#define __GENERICSWITCH_H__

#include "HEAR_core/Block.hpp"

namespace HEAR {
template <class T>
class GenericSwitch : public Block {
private :
    bool _triggered = false;
    InputPort<T> *_input_port;
    OutputPort<T> *_default_port, *_other_port;
public:
    enum IP{COM};
    enum OP{NC, NO};
    GenericSwitch(int b_uid);
    void process();
    void update(UpdateMsg* u_msg) override;

};

template <class T>
GenericSwitch<T>::GenericSwitch(int b_uid) : Block(BLOCK_ID::SWITCH, b_uid){
    _input_port = createInputPort<T>(IP::COM, "COM");
    _default_port = createOutputPort<T>(OP::NC, "NC"); _default_port->write(0);
    _other_port = createOutputPort<T>(OP::NO, "NO"); _other_port->write(0);
}

template <class T>
void GenericSwitch<T>::process(){
    T data;
    _input_port->read(data);
    if(_triggered){
        _other_port->write(data);
    }
    else {
        _default_port->write(data);
    }
}

template <class T>
void GenericSwitch<T>::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::SWITCH_TRIG){
        switch (((SwitchMsg*)u_msg)->sw_state)
        {
        case SWITCH_STATE::ON :
            _triggered = true;
            break;
        case SWITCH_STATE::OFF :
            _triggered = false;
            break;
        case SWITCH_STATE::TOGGLE :
            _triggered == true? _triggered = false : _triggered = true;
            break;
        default:
            break;
        }
    }
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        _triggered = ((BoolMsg*)u_msg)->data;
    }
}

}

#endif // __GENERICSWITCH_H__