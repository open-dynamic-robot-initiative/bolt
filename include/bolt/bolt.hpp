/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Bolt biped low level drivers.
 */

#pragma once

#include <math.h>

#include <Eigen/Eigen>

#include "slider_box/serial_reader.hpp"
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/robot.hpp"

namespace Eigen
{
/** @brief Eigen shortcut for vector of size 6. */
typedef Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

namespace bolt
{
#define BOLT_NB_MOTOR_BOARD 3
#define BOLT_NB_MOTOR 6
#define BOLT_NB_SLIDER 4

/** @brief Control state of the robot. */
enum BoltControlState
{
    initial,
    ready,
    calibrate
};

/**
 * @brief Driver for the Bolt biped robot.
 */
class Bolt
{
public:
    /**
     * @brief Bolt is the constructor of the class.
     */
    Bolt();

    /**
     * @brief initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0
     */
    void initialize(const std::string& network_id);

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    void send_target_joint_torque(
        const Eigen::Ref<const Eigen::Vector6d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Wait until the hardware is ready to be controlled.
     */
    void wait_until_ready();

    /**
     * @brief Fill attitude quaternion.
     */
    void fill_base_attitude_quaternion();

    /**
     * @brief Request calibration of the joints by moving to the next joint
     * index position. The control is made inside the send_target_joint_torque.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true
     * @return false
     */
    void request_calibration(
        const Eigen::Ref<const Eigen::VectorXd> home_offset_rad);

    /**
     * @brief Request calibration of the joints by moving to the next joint
     * index position. The control is made inside the send_target_joint_torque.
     * Use the yaml information in order to get the offset from the nearest
     * motor index.
     *
     * @return true
     * @return false
     */
    void request_calibration();

    /**
     * Sensor Data
     */

    /**
     * @brief get_joint_positions
     * @return  the joint angle of each module
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector6d> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
     * @brief get_joint_velocities
     * @return the joint velocities
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector6d> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
     * @brief get_joint_torques
     * @return the joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector6d> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
     * @brief get_joint_torques
     * @return the target joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.

     */
    const Eigen::Ref<const Eigen::Vector6d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_accelerometer()
    {
        return base_accelerometer_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_gyroscope()
    {
        return base_gyroscope_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector4d> get_base_attitude()
    {
        return base_attitude_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_linear_acceleration()
    {
        return base_linear_acceleration_;
    }

    /**
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> >
    get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> >
    get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const Eigen::Ref<const Eigen::Matrix<bool, BOLT_NB_MOTOR_BOARD, 1> >
    get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const Eigen::Ref<const Eigen::Matrix<int, BOLT_NB_MOTOR_BOARD, 1> >
    get_motor_board_errors()
    {
        return motor_board_errors_;
    }

    /**
     * @brief has_error
     * @return Returns true if the robot hardware has an error, false otherwise.
     */
    bool has_error() const
    {
        return robot_->HasError();
    }

    /*
     * Additional data
     */

    /**
     * @brief Get the slider positions in [0, 1]
     *
     * @return Eigen::Ref<Eigen::Matrix<double, BOLT_NB_SLIDER, 1> >
     */
    const Eigen::Ref<const Eigen::Matrix<double, BOLT_NB_SLIDER, 1> >
    get_slider_positions()
    {
        return slider_positions_;
    }

    /**
     * @brief Get the active estop value
     *
     * @return true if the button is pressed.
     * @return false otherwize.
     */
    bool get_active_estop() const
    {
        return active_estop_;
    }

    /**
     * @brief is_calibrating()
     * @return Returns true if the calibration procedure is running right now.
     */
    bool is_calibrating()
    {
        return (control_state_ == BoltControlState::calibrate) || calibrate_request_;
    }

private:
    /*
     * Hardware status
     */

    /** @brief Motor status (enabled/disabled). */
    Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> motor_enabled_;

    /** @brief Motor readiness to receive commands (ready/not ready). */
    Eigen::Matrix<bool, BOLT_NB_MOTOR, 1> motor_ready_;

    /** @brief Motor Board status (enabled/disabled). */
    Eigen::Matrix<bool, BOLT_NB_MOTOR_BOARD, 1> motor_board_enabled_;

    /** @brief Motor Board Error code (int).
     */
    Eigen::Matrix<int, BOLT_NB_MOTOR_BOARD, 1> motor_board_errors_;

    /*
     * Joint data
     */

    /** @brief Joint positions. */
    Eigen::Vector6d joint_positions_;

    /** @brief Joint velocities. */
    Eigen::Vector6d joint_velocities_;

    /** @brief Joint torques. */
    Eigen::Vector6d joint_torques_;

    /** @brief Target joint torques, reference from the controller. */
    Eigen::Vector6d joint_target_torques_;

    /*
     * Additional data
     */

    /** @brief Name of the network lan: Left column in ifconfig output. */
    std::string network_id_;

    /** @brief Slider position in [0, 1]. */
    Eigen::Matrix<double, BOLT_NB_SLIDER, 1> slider_positions_;

    /** @brief E-stop from the slider box. */
    bool active_estop_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_accelerometer_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_gyroscope_;

    /** @brief base accelerometer. */
    Eigen::Vector4d base_attitude_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_linear_acceleration_;

    /** @brief Integers from the serial port.
     * - 4 sliders in [0, 1024]
     * - emergency stop
     */
    std::vector<int> slider_box_data_;

    /*
     * Controllers
     */

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;

    /*
     * Finite state machine of the controbolller.
     */

    /** @brief Control state Initial, Ready, Calibration */
    BoltControlState control_state_;

    /** @brief Check if the user called for the joint calibration. */
    bool calibrate_request_;

    /** @brief Number of time we acquire the sensor readings, this is used
     *  to prevent spamming prints */
    long int nb_time_we_acquired_sensors_;

    /*
     * Drivers communication objects
     */

    /**
     * @brief Robot complete interface.
     * Wrapper around the MasterBoardInterface.
     *
     * PC <------------------> main board <-------> Motor Board
     *       Ethernet/Wifi                   SPI
     */
    std::shared_ptr<odri_control_interface::Robot> robot_;

    /**
     * @brief Reader for serial port to read arduino slider values.
     */
    std::shared_ptr<slider_box::SerialReader> serial_reader_;
};

}  // namespace bolt
