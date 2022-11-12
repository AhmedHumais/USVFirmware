
#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"
#include "Navio2Imu.hpp"

#define BIG_HEXA

namespace HEAR
{
class StateEstimatorNodelet : public nodelet::Nodelet{

public:
    StateEstimatorNodelet() = default;
    virtual ~StateEstimatorNodelet();

private:
    const int FREQUENCY = 200;
    virtual void onInit();
    
    RosSystem* sys;
    ROSUnit_PoseProvider* providers; 
};
    
} 