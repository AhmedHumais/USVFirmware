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
    // enable or disable heartbeat
    usv_driver_blk->_hb_enabled = false;
    //
    auto rc_driver = new RCin(0);
    auto usv_rc_controller = new UsvRadioController(0); sys->addBlock(usv_rc_controller, "USV RC Controller");
    auto rc_override = new RCManualOverride(0); sys->addBlock(rc_override, "RC Override");
    auto manual_sw = new GenericInvertedSwitch<std::vector<float>>(0); sys->addBlock(manual_sw, "Manual Control Switch");

    sys->createSub(TYPE::FloatVec, "/usv_cmd", manual_sw->getInputPort<std::vector<float>>(GenericInvertedSwitch<void>::IP::NO));
    sys->connect(rc_driver->getOutputPort<std::vector<int>>(RCin::OP::RC_COMMAND), 
                rc_override->getInputPort<std::vector<int>>(RCManualOverride::IP::RC_RAW));

    sys->connect(usv_rc_controller->getOutputPort<std::vector<float>>(UsvRadioController::OP::CONTROL_CMD), 
                manual_sw->getInputPort<std::vector<float>>(GenericInvertedSwitch<void>::IP::NC));

    sys->connect(manual_sw->getOutputPort<std::vector<float>>(GenericInvertedSwitch<void>::OP::COM),
                rc_override->getInputPort<std::vector<float>>(RCManualOverride::IP::CONTROL_CMD));

    sys->connect(usv_rc_controller->getOutputPort<std::vector<float>>(UsvRadioController::OP::CONTROL_CMD),
                rc_override->getInputPort<std::vector<float>>(RCManualOverride::IP::RC_CMD));
    
    sys->connect(rc_override->getOutputPort<std::vector<float>>(RCManualOverride::OP::CMD_OUT),
                usv_driver_blk->getInputPort<std::vector<float>>(UsvDriver::IP::CONTROL_CMD));
    
    sys->connect(usv_driver_blk->getOutputPort<int>(UsvDriver::OP::LEFT_TH_CMD), th_left_blk->getInputPort<int>(Thruster::IP::INP));
    sys->connect(usv_driver_blk->getOutputPort<int>(UsvDriver::OP::RIGHT_TH_CMD), th_right_blk->getInputPort<int>(Thruster::IP::INP));

    sys->createPub(TYPE::IntVec, "/rc_out", rc_driver->getOutputPort<std::vector<int>>(RCin::OP::RC_COMMAND));
    sys->createPub(TYPE::FloatVec, "/cmd_out", rc_override->getOutputPort<std::vector<float>>(RCManualOverride::OP::CMD_OUT));
    sys->createPub(TYPE::Int, "/thruster_left_out", usv_driver_blk->getOutputPort<int>(UsvDriver::OP::LEFT_TH_CMD));
    sys->createPub(TYPE::Int, "/thruster_right_out", usv_driver_blk->getOutputPort<int>(UsvDriver::OP::RIGHT_TH_CMD));    
    
    sys->createResetTrigger("/reset_manual_override", rc_override);


}


}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<HEAR::UsvNode>("usv_driver_node");
    node_ptr->run_sys(node_ptr);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}