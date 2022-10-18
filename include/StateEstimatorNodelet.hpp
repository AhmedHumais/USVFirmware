
#include "HEAR_ROS2/RosSystem.hpp"
#include "HEAR_ROS2/Custom/ROSUnit_PoseProvider.hpp"
#include "Navio2Imu.hpp"

#define BIG_HEXA

namespace HEAR
{
class StateEstimatorNodelet : public rclcpp::Node{

public:
    StateEstimatorNodelet(std::string node_name) : Node(node_name){
        this->run_sys();
    }
    ~StateEstimatorNodelet();

private:
    const int FREQUENCY = 200;

    void run_sys();
    
    RosSystem* sys;
    ROSUnit_PoseProvider* providers; 
};
    
} 