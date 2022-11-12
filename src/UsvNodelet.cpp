#include "UsvNodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(HEAR::UsvNodelet, nodelet::Nodelet)

namespace HEAR{

UsvNodelet::~UsvNodelet(){
    delete sys;
}

void UsvNodelet::onInit(){

    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle pnh(getPrivateNodeHandle());

    sys = new RosSystem(nh, pnh, FREQUENCY, "usv_node");

    auto th_left_blk = new Thruster(0, 0);  sys->addBlock(th_left_blk, "LeftTh");
    auto th_right_blk = new Thruster(1,1);  sys->addBlock(th_right_blk, "RightTh");
    auto usv_driver_blk = new UsvDriver(2); sys->addBlock(usv_driver_blk, "USV Driver");
    // enable or disable heartbeat
    usv_driver_blk->_hb_enabled = false;
    //
    auto rc_driver = new RCin(3, 12); sys->addBlock(rc_driver, "RC Driver");
    auto usv_rc_controller = new UsvRadioController(4); sys->addBlock(usv_rc_controller, "USV RC Controller");
    usv_rc_controller->setChanMap(1, 0);
    usv_rc_controller->reverse_fwd = true; 
    usv_rc_controller->reverse_yaw = true;
    auto rc_override = new RCManualOverride(5); sys->addBlock(rc_override, "RC Override");
    rc_override->setRCoverrideChan(4);
    auto manual_sw = new GenericInvertedSwitch<std::vector<float>>(6); sys->addBlock(manual_sw, "Manual Control Switch");

    
    sys->connect(rc_driver->getOutputPort<std::vector<int>>(RCin::OP::RC_COMMAND), 
                rc_override->getInputPort<std::vector<int>>(RCManualOverride::IP::RC_RAW));
    sys->connect(rc_driver->getOutputPort<std::vector<int>>(RCin::OP::RC_COMMAND), 
                usv_rc_controller->getInputPort<std::vector<int>>(UsvRadioController::IP::RC_COMMAND));

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

    sys->createSub(TYPE::FloatVec, "/usv_cmd", manual_sw->getInputPort<std::vector<float>>(GenericInvertedSwitch<void>::IP::NO));

    sys->createPub(TYPE::IntVec, "/rc_out", rc_driver->getOutputPort<std::vector<int>>(RCin::OP::RC_COMMAND));
    sys->createPub(TYPE::FloatVec, "/cmd_out", rc_override->getOutputPort<std::vector<float>>(RCManualOverride::OP::CMD_OUT));
    sys->createPub(TYPE::Int, "/thruster_left_out", usv_driver_blk->getOutputPort<int>(UsvDriver::OP::LEFT_TH_CMD));
    sys->createPub(TYPE::Int, "/thruster_right_out", usv_driver_blk->getOutputPort<int>(UsvDriver::OP::RIGHT_TH_CMD));    
    
    sys->createResetTrigger("/reset_manual_override", rc_override);
    sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/arm_usv", usv_driver_blk);
    auto rst_th_trig = sys->createResetTrigger("/reinitialize_thrusters");
    sys->connectExternalTrigger(rst_th_trig, th_left_blk);
    sys->connectExternalTrigger(rst_th_trig, th_right_blk);
    sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/enable_auto", manual_sw);
    sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/calibrate_rc", usv_rc_controller);

    sys->start();

    std::cout << "Created all blocks\n";

}


}
