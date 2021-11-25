/**
 * \file dgm_bolt.cpp
 * \brief The hardware wrapper of the bolt robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "bolt/dgm_bolt.hpp"

#include "dynamic_graph_manager/ros.hpp"

namespace bolt
{
DGMBolt::DGMBolt()
{
    was_in_safety_mode_ = false;
}

DGMBolt::~DGMBolt()
{
}

bool DGMBolt::is_in_safety_mode()
{
    // Check if any card is in an error state.
    if (bolt_.has_error()) {
      was_in_safety_mode_ = true;
      static int counter = 0;
      if (counter % 2000 == 0) {
        printf("DGMBolt: Going into safe mode as motor card reports error.\n");
      }
      counter += 1;
    }

    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode())
    {
      static int counter = 0;
      was_in_safety_mode_ = true;
      if (counter % 2000 == 0)
      {
        printf("DGMBolt: is_in_safety_mode.\n");
      }
      counter++;
    }
    return was_in_safety_mode_;
  }


void DGMBolt::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters
     */

    // get the hardware communication ros node handle
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** initialize the user commands */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMBolt::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    bolt_.initialize(network_id);
}

void DGMBolt::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    bolt_.acquire_sensors();

    /**
     * Joint data
     */
    map.at("joint_positions") = bolt_.get_joint_positions();
    map.at("joint_velocities") = bolt_.get_joint_velocities();
    map.at("joint_torques") = bolt_.get_joint_torques();
    map.at("joint_target_torques") = bolt_.get_joint_target_torques();

    /**
     * Additional data
     */
    map.at("base_accelerometer") = bolt_.get_base_accelerometer();
    map.at("base_gyroscope") = bolt_.get_base_gyroscope();
    map.at("base_attitude") = bolt_.get_base_attitude();
    map.at("base_linear_acceleration") = bolt_.get_base_linear_acceleration();
    map.at("slider_positions") = bolt_.get_slider_positions();

    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> > motor_enabled =
        bolt_.get_motor_enabled();
    Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> > motor_ready =
        bolt_.get_motor_ready();
    Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR_BOARD, 1> >
        motor_board_enabled = bolt_.get_motor_board_enabled();
    Eigen::Ref<const Eigen::Matrix<int, BOLT_NB_MOTOR_BOARD, 1> >
        motor_board_errors = bolt_.get_motor_board_errors();

    for (unsigned i = 0; i < BOLT_NB_MOTOR; ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (unsigned i = 0; i < BOLT_NB_MOTOR_BOARD; ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMBolt::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // here we need to perform and internal copy. Otherwise the compilator
        // complains
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot
        bolt_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMBolt::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

void DGMBolt::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMBolt::calibrate_joint_position, this));

    // return whatever the user want
    res->sanity_check = true;
}

void DGMBolt::calibrate_joint_position()
{
    bolt_.request_calibration();
}

}  // namespace bolt
