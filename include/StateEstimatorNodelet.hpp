
#include "HEAR_ROS2/RosSystem.hpp"
#include "HEAR_ROS2/Custom/ROSUnit_PoseProvider.hpp"
#include "Navio2Imu.hpp"

#include <iostream>

#define BIG_HEXA

namespace HEAR
{
class StateEstimatorNodelet : public rclcpp::Node{

public:
    StateEstimatorNodelet(std::string node_name) : Node(node_name){
    }
    ~StateEstimatorNodelet();
    void run_sys(rclcpp::Node::SharedPtr);

private:
    const int FREQUENCY = 200;

    
    RosSystem* sys;
    ROSUnit_PoseProvider* providers; 
};
    
} 