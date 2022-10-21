#ifndef __USV_NODE_H__
#define __USV_NODE_H__

#include "HEAR_ROS2/RosSystem.hpp"
#include "UsvModules.hpp"
#include "NavioDrivers.hpp"

namespace HEAR{

class UsvNode : public rclcpp::Node {

public:
    UsvNode(std::string node_name) : Node(node_name){
    }
    ~UsvNode();
    void run_sys(rclcpp::Node::SharedPtr);

private:
    const int FREQUENCY = 200;

    
    RosSystem* sys;
};

}
#endif // __USV_NODE_H__