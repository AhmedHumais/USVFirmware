#ifndef __STATEESTIMATORNODELET_H__
#define __STATEESTIMATORNODELET_H__


#include "StateEstimatorNodelet.hpp"
#include <pluginlib/class_list_macros.h>

//#define XSENS

PLUGINLIB_EXPORT_CLASS(HEAR::StateEstimatorNodelet, nodelet::Nodelet)

namespace HEAR
{
    StateEstimatorNodelet::~StateEstimatorNodelet(){
        delete sys;
    }
    void StateEstimatorNodelet::onInit(){
    {
        std::cout << "system creating ..." << std::endl;

        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        sys = new RosSystem(nh, pnh, FREQUENCY, "State Estimation System");
        
        providers = new ROSUnit_PoseProvider (nh);
        auto opti_pos_port = sys->createExternalInputPort<Vector3D<float>>("Pos_port");
        auto opti_vel_port = sys->createExternalInputPort<Vector3D<float>>("Vel_port");
        auto opti_ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_port");

        auto opti_port = providers->registerOptiPose("/Robot_1/pose");
        sys->connectExternalInput(opti_pos_port, opti_port[0]);
        sys->connectExternalInput(opti_vel_port, opti_port[1]);
        sys->connectExternalInput(opti_ori_port, opti_port[2]);

        // Setting up publishers for opti track
        sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/pos", ((Block *)opti_pos_port)->getOutputPort<Vector3D<float>>(0));
        sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/vel", ((Block *)opti_vel_port)->getOutputPort<Vector3D<float>>(0));
        sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/ori", ((Block *)opti_ori_port)->getOutputPort<Vector3D<float>>(0));

        auto filt_angle_rate = sys->createBlock(BLOCK_ID::BW_FILT2, "Filt_angle_rate", TYPE::Float3);
        ((BWFilter2<Vector3D<float>> *)filt_angle_rate)->setCoeff(BWFilt2_coeff::coeff_N200C60);

        sys->createPub(TYPE::Float3, "/angle_rate", filt_angle_rate->getOutputPort<Vector3D<float>>(0));

#ifdef XSENS

        auto ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_port");
        auto angle_rate_port = sys->createExternalInputPort<Vector3D<float>>("Angle_rt_port");
        sys->connectExternalInput(ori_port, providers->registerImuOri("/filter/quaternion"));
        sys->connectExternalInput(angle_rate_port, providers->registerImuAngularRate("/imu/angular_velocity"));
        sys->connectExternalInput(angle_rate_port, filt_angle_rate->getInputPort<Vector3D<float>>(0));

        sys->createPub(TYPE::Float3, "/imu/ori", ((Block *)ori_port)->getOutputPort<Vector3D<float>>(0));

#else
        std::cout << "creating IMU" << std::endl;
        auto imu_navio = new Navio2Imu(0, FREQUENCY);
        // auto rotate = new FromHorizon(0);
        // auto rot_ang = sys->createBlock(BLOCK_ID::CONSTANT, "IMU offset angle", TYPE::Float);((Constant<float>*)rot_ang)->setValue(-M_PI_2);
        // sys->addBlock(rotate, "Rotate offset");
        // sys->addBlock(to_rot, "Quat to Rot");
        // sys->addBlock(to_eul, "Rot to Eul");
        sys->addBlock(imu_navio, "Navio IMU");
        // sys->createPub(TYPE::Float3, "/navio/quaternion", imu_navio->getOutputPort<Vector3D<float>>(Navio2Imu::OP::RPY));
        // sys->connect(imu_navio->getOutputPort<tf2::Quaternion>(Navio2Imu::OP::QUAT), to_rot->getInputPort<tf2::Quaternion>(Quat2Rot::IP::QUAT));
        // sys->connect(to_rot->getOutputPort<tf2::Matrix3x3>(Quat2Rot::OP::ROT_MAT), to_eul->getInputPort<tf2::Matrix3x3>(Rot2Eul::IP::ROT_MAT));
        // sys->connect(to_eul->getOutputPort<Vector3D<float>>(Rot2Eul::OP::EUL_ANGLES), rotate->getInputPort<Vector3D<float>>(FromHorizon::IP::INP_VEC));
        // sys->connect(rot_ang->getOutputPort<float>(Constant<float>::OP::OUTPUT), rotate->getInputPort<float>(FromHorizon::IP::YAW));
        sys->connect(imu_navio->getOutputPort<Vector3D<float>>(Navio2Imu::OP::GYRO), filt_angle_rate->getInputPort<Vector3D<float>>(0));

        sys->createPub(TYPE::Float3, "/imu/ori", imu_navio->getOutputPort<Vector3D<float>>(Navio2Imu::OP::RPY));
        sys->createPub(TYPE::Float3, "/navio/acc", imu_navio->getOutputPort<Vector3D<float>>(Navio2Imu::OP::ACC));
        sys->createPub(TYPE::Float3, "/navio/gyro", imu_navio->getOutputPort<Vector3D<float>>(Navio2Imu::OP::GYRO));

        /// Calibrating IMU
        std::cout << "starting Calibration" << std::endl;
        imu_navio->calibrateIMU();
///
#endif

        // adding battery monitor
        // auto bat_mon = new BatteryMonitor(0);
        // sys->addBlock(bat_mon, "Battery Monitor");
        // sys->createPub("/battery_voltage",bat_mon->getOutputPort<float>(BatteryMonitor::OP::BAT_VOLT));
        //

        sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/enable_inner_filter", filt_angle_rate); // change name to enable gyro filter

        sys->start();

        std::cout << "Created all blocks\n";
    }

}

}

#endif // __STATEESTIMATORNODELET_H__