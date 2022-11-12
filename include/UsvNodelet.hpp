#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "UsvModules.hpp"
#include "NavioDrivers.hpp"


namespace HEAR
{
class UsvNodelet : public nodelet::Nodelet{

public:
    UsvNodelet() = default;
    virtual ~UsvNodelet();

private:
    const int FREQUENCY = 200;
    virtual void onInit();
    
    RosSystem* sys;
};

}