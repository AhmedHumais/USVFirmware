#include "usv_node.hpp"

namespace HEAR{

UsvNode::~UsvNode(){
    delete sys;
}

void UsvNode::run_sys(rclcpp::Node::SharedPtr node_p){

    sys = new RosSystem(node_p, 200, "usv_node");

    auto th_left_blk = new Thruster(0, 0);  sys->addBlock(th_left_blk, "Left Thruster 0");
    auto th_right_blk = new Thruster(0,1);  sys->addBlock(th_right_blk, "Right Thruster 1");
    auto usv_driver_blk = new UsvDriver(0); sys->addBlock(usv_driver_blk, "USV Driver");
    auto usv_rc_controller = new UsvRadioController(0); sys->addBlock(usv_rc_controller, "USV RC Controller");
    auto rc_override = new RCManualOverride(0);
    auto manual_sw = new 


}


}