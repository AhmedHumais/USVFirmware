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

}

#endif // __GENERICSWITCH_H__