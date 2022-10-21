#ifndef __GENERICINVERTEDSWITCH_H__
#define __GENERICINVERTEDSWITCH_H__

#include "HEAR_core/Block.hpp"

namespace HEAR {
template <class T>
class GenericInvertedSwitch : public Block {
private :
    bool _triggered = false;
    InputPort<T> *_default_port, *_other_port;
    OutputPort<T> *_output_port;
public:
    enum IP{NC, NO};
    enum OP{COM};
    GenericInvertedSwitch(int b_uid);
    void process();
    void update(UpdateMsg* u_msg) override;

};

template <class T>
GenericInvertedSwitch<T>::GenericInvertedSwitch(int b_uid) : Block(BLOCK_ID::SWITCH, b_uid){
    _default_port = createInputPort<T>(IP::NC, "NC");
    _other_port = createInputPort<T>(IP::NO, "NO");
    _output_port = createOutputPort<T>(OP::COM, "COM");
}

template <class T>
void GenericInvertedSwitch<T>::process(){
    T data;
    if(_triggered){
        _other_port->read(data);
    }
    else {
        _default_port->read(data);
    }
    _output_port->write(data);
}

template <class T>
void GenericInvertedSwitch<T>::update(UpdateMsg* u_msg){
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
#endif // __GENERICINVERTEDSWITCH_H__