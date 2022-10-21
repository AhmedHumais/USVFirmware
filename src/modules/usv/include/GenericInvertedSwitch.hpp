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

}
#endif // __GENERICINVERTEDSWITCH_H__