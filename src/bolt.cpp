/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implement the Bolt class.
 */

#include "bolt/bolt.hpp"

#include <cmath>

#include "odri_control_interface/utils.hpp"

namespace bolt
{
Bolt::Bolt()
{
    /**
     * Hardware status
     */
    for (unsigned i = 0; i < motor_enabled_.size(); ++i)
    {
        motor_enabled_[i] = false;
        motor_ready_[i] = false;
    }
    for (unsigned i = 0; i < motor_board_enabled_.size(); ++i)
    {
        motor_board_enabled_[0] = false;
        motor_board_errors_[0] = 0;
    }

    /**
     * Joint data
     */
    joint_positions_.setZero();
    joint_velocities_.setZero();
    joint_torques_.setZero();
    joint_target_torques_.setZero();

    /**
     * Additional data
     */

    // Network infos.
    network_id_ = "";

    // Slider bos infos.
    slider_positions_.setZero();
    active_estop_ = true;  // By default assume the estop is active.
    slider_box_data_.resize(BOLT_NB_SLIDER + 1, 0);  // 4 sliders + 1 e-stop.

    // imu infos
    base_accelerometer_.setZero();
    base_gyroscope_.setZero();
    base_attitude_.setZero();
    base_linear_acceleration_.setZero();
    base_attitude_quaternion_.setZero();

    // Finite state machine for the control
    control_state_ = BoltControlState::initial;
    calibrate_request_ = false;
    is_calibrating_ = false;
}

void Bolt::initialize(const std::string& network_id)
{
    // Network info.
    network_id_ = network_id;

    // Main driver interface.
    robot_ = odri_control_interface::RobotFromYamlFile(
        network_id_, ODRI_CONTROL_INTERFACE_YAML_PATH);

    // Use a serial port to read slider values.
    serial_reader_ = std::make_shared<blmc_drivers::SerialReader>(
        "serial_port", BOLT_NB_SLIDER + 1);

    // Initialize the robot.
    robot_->Init();
}

void Bolt::acquire_sensors()
{
    // Acquire the data.
    robot_->ParseSensorData();

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = robot_->joints->GetPositions();
    // acquire the joint velocities
    joint_velocities_ = robot_->joints->GetVelocities();
    // acquire the joint torques
    joint_torques_ = robot_->joints->GetMeasuredTorques();
    // acquire the target joint torques
    joint_target_torques_ = robot_->joints->GetSentTorques();

    /**
     * Additional data
     */
    base_accelerometer_ = robot_->imu->GetAccelerometer();
    base_gyroscope_ = robot_->imu->GetGyroscope();
    base_attitude_ = robot_->imu->GetAttitudeEuler();
    base_attitude_quaternion_ = robot_->imu->GetAttitudeQuaternion();
    base_linear_acceleration_ = robot_->imu->GetLinearAcceleration();

    // acquire the slider positions
    if (serial_reader_->fill_vector(slider_box_data_) > 10)
    {
        robot_->ReportError(
            "The slider box is not responding correctly, "
            "10 iteration are missing.");
    }
    for (unsigned i = 0; i < slider_positions_.size(); ++i)
    {
        // acquire the slider
        slider_positions_(i) = double(slider_box_data_[i + 1]) / 1024.;
    }
    // acquire the e-stop from the slider box
    active_estop_ = slider_box_data_[0] == 0;

    /**
     * The different status.
     */

    // motor board status
    motor_board_enabled_ = robot_->joints->GetMotorDriverEnabled();
    motor_board_errors_ = robot_->joints->GetMotorDriverErrors();
    // motors status
    motor_enabled_ = robot_->joints->GetEnabled();
    motor_ready_ = robot_->joints->GetReady();

    /*
     * Safety check
     */
    if (base_attitude_(1) < 0.6 && base_attitude_(1) > -0.6 &&
        (base_attitude_(0) < -2.5 || base_attitude_(0) > 2.5))
    {
        robot_->ReportError("Base attitude not in the defined parameter.");
    }
}

void Bolt::send_target_joint_torque(
    const Eigen::Ref<const Eigen::Vector6d> target_joint_torque)
{
    robot_->joints->SetTorques(target_joint_torque);

    switch (control_state_)
    {
        case BoltControlState::initial:
            robot_->joints->SetZeroCommands();
            if (!robot_->IsTimeout() && !robot_->IsAckMsgReceived())
            {
                robot_->SendInit();
            }
            else if (!robot_->IsReady())
            {
                robot_->SendCommand();
            }
            else
            {
                control_state_ = BoltControlState::ready;
            }
            break;

        case BoltControlState::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                control_state_ = BoltControlState::calibrate;
                is_calibrating_ = true;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case BoltControlState::calibrate:
            if (calib_ctrl_->Run())
            {
                control_state_ = BoltControlState::ready;
                is_calibrating_ = false;
            }
            robot_->SendCommand();
            break;
    }
}

void Bolt::request_calibration(
    const Eigen::Ref<const Eigen::VectorXd> home_offset_rad)
{
    printf("Bolt::calibrate called\n");
    calib_ctrl_->UpdatePositionOffsets(home_offset_rad);
    calibrate_request_ = true;
}

void Bolt::request_calibration()
{
    printf("Bolt::calibrate called\n");
    calibrate_request_ = true;
}

}  // namespace bolt
