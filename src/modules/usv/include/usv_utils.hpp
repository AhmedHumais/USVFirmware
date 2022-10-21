#ifndef __USV_UTILS_H__
#define __USV_UTILS_H__

#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

enum USV_MSG_TYPE{
        THRUSTER_RESET,
        THRUSTER_ENABLE
};

class UsvMsg : UpdateMsg{
public:
    UPDATE_MSG_TYPE getType() const {
        return UPDATE_MSG_TYPE::USVMSG;
    }
    virtual USV_MSG_TYPE getUsvMsgType() const =0;
    virtual  UsvMsg* copyUsvMsg() const=0;
    UpdateMsg* copy() const{
        return this->copyUsvMsg();
    } 
};

class ThrusterMsg : UsvMsg{
private:
    USV_MSG_TYPE msg_type = THRUSTER_RESET;
public:
    bool enable = true;
    void set_msg_type(USV_MSG_TYPE _type){
        msg_type = _type;
    }
    USV_MSG_TYPE getUsvMsgType() const{
        return msg_type;
    }
    UsvMsg* copyUsvMsg() const{
        auto Copy = new ThrusterMsg;
        *Copy = *this;
        return Copy;
    }
};

}
#endif // __USV_UTILS_H__