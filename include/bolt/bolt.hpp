/**

 * @version 0.1
 * @date 2020-05-07
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <math.h>
#include "blmc_drivers/devices/spi_motor_board.hpp"
#include "blmc_drivers/serial_reader.hpp"
#include "blmc_drivers/blmc_joint_module.hpp"
#include "blmc_robots/common_header.hpp"

namespace bolt
{
#define BOLT_NB_MOTOR_BOARD 3
#define BOLT_NB_MOTOR 6
#define BOLT_NB_SLIDER 4

/**
 * @brief Vector6d shortcut for the eigen vector of size BOLT_NB_MOTOR.
 */
typedef Eigen::Matrix<double, BOLT_NB_MOTOR, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

/**
 * @brief Driver for the Bolt biped robot.
 *
 * Map of the joints:
 *
 * +----------|------------|---------|------------|-------------+
 * + joint_id | joint_name | udriver | motor_port | motor_index +
 * +==========|============|=========|============|=============+
 * +    0     |   L_HAA    |   0     |    0       |     0       +
 * +----------|------------|---------|------------|-------------+
 * +    1     |   L_HFE    |   1     |    1       |     3       +
 * +----------|------------|---------|------------|-------------+
 * +    2     |   L_KFE    |   1     |    0       |     2       +
 * +----------|------------|---------|------------|-------------+
 * +    3     |   R_HAA    |   0     |    1       |     1       +
 * +----------|------------|---------|------------|-------------+
 * +    4     |   R_HFE    |   2     |    1       |     5       +
 * +----------|------------|---------|------------|-------------+
 * +    5     |   R_KFE    |   2     |    0       |     4       +
 * +----------|------------|---------|------------|-------------+
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
     * @brief Sets the maximum joint torques.
     */
    void set_max_joint_torques(const double& max_joint_torques)
    {
        max_joint_torques_.fill(max_joint_torques);
    }

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    void send_target_joint_torque(
        const Eigen::Ref<Vector6d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Fill attitude quaternion.
     */
    void fill_base_attitude_quaternion();

    /**
     * @brief Calibrate the joints by moving to the next joint index position.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true
     * @return false
     */
    bool calibrate(const Vector6d& home_offset_rad);

    /**
     * Joint properties
     */

    /**
     * @brief get_motor_inertias
     * @return the motor inertias
     */
    const Eigen::Ref<Vector6d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants
     * @return the torque constants of each motor
     */
    const Eigen::Ref<Vector6d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return  the joint gear ratios
     */
    const Eigen::Ref<Vector6d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_max_torque
     * @return the max torque that has been hardcoded in the constructor of this
     * class. TODO: parametrize this via yaml or something else.
     */
    const Eigen::Ref<Vector6d> get_motor_max_current()
    {
        return motor_max_current_;
    }

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
    const Eigen::Ref<Vector6d> get_joint_positions()
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
    const Eigen::Ref<Vector6d> get_joint_velocities()
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
    const Eigen::Ref<Vector6d> get_joint_torques()
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
    const Eigen::Ref<Vector6d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_joint_encoder_index
     * @return the position of the index of the encoders a the motor level
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector6d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector3d> get_base_accelerometer()
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
    const Eigen::Ref<Vector3d> get_base_gyroscope()
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
    const Eigen::Ref<Vector3d> get_base_attitude()
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
    const Eigen::Ref<Vector3d> get_base_linear_acceleration()
    {
        return base_linear_acceleration_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector4d> get_base_attitude_quaternion()
    {
        return base_attitude_quaternion_;
    }

    /**
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, BOLT_NB_MOTOR>& get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, BOLT_NB_MOTOR>& get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<bool, BOLT_NB_MOTOR_BOARD>& get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<int, BOLT_NB_MOTOR_BOARD>& get_motor_board_errors()
    {
        return motor_board_errors_;
    }

    /**
     * @brief has_error
     * @return Returns true if the robot hardware has an error, false otherwise.
     */
    bool has_error() const
    {
        for (const auto& error_code : motor_board_errors_)
        {
            if (error_code != 0)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Additional data
     */

    /**
     * @brief Get the slider positions in [0, 1]
     *
     * @return Eigen::Ref<Eigen::Matrix<double, BOLT_NB_SLIDER, 1> >
     */
    const Eigen::Ref<Eigen::Matrix<double, BOLT_NB_SLIDER, 1> >
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

private:
    /**
     * Joint properties
     */
    Vector6d motor_inertias_;         /**! motors inertia. */
    Vector6d motor_torque_constants_; /**! DCM motor torque constants. */
    Vector6d joint_gear_ratios_;      /**! joint gear ratios (9). */
    Vector6d motor_max_current_;    /**! Max appliable current before the robot
                                         shutdown. */
    Vector6d joint_zero_positions_; /**! Offset to the theoretical "0" pose. */
    /** @brief Max joint torques (Nm) */
    Eigen::Array<double, BOLT_NB_MOTOR, 1> max_joint_torques_;
    /** @brief Security margin on the saturation of the control. */
    static const double max_joint_torque_security_margin_;

    /**
     * Hardware status
     */
    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, BOLT_NB_MOTOR> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, BOLT_NB_MOTOR> motor_ready_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    std::array<bool, BOLT_NB_MOTOR_BOARD> motor_board_enabled_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    std::array<int, BOLT_NB_MOTOR_BOARD> motor_board_errors_;

    /**
     * Joint data
     */

    /** @brief Joint positions. */
    Vector6d joint_positions_;

    /** @brief Joint velocities. */
    Vector6d joint_velocities_;

    /** @brief Joint torques. */
    Vector6d joint_torques_;

    /** @brief Target joint torques, reference from the controller. */
    Vector6d joint_target_torques_;

    /** @brief Joint encoder index. */
    Vector6d joint_encoder_index_;

    /**
     * Additional data
     */

    /** @brief Map the joint id to the motor board id, @see Bolt description. */
    std::array<int, BOLT_NB_MOTOR> map_joint_id_to_motor_board_id_;

    /** @brief Map the joint id to the motor port id, @see Bolt description. */
    std::array<int, BOLT_NB_MOTOR> map_joint_id_to_motor_port_id_;

    /** @brief Name of the network lan: Left column in ifconfig output. */
    std::string network_id_;

    /** @brief Slider position in [0, 1]. */
    Eigen::Matrix<double, BOLT_NB_SLIDER, 1> slider_positions_;

    /** @brief E-stop from the slider box. */
    bool active_estop_;

    /** @brief base accelerometer. */
    Vector3d base_accelerometer_;

    /** @brief base accelerometer. */
    Vector3d base_gyroscope_;

    /** @brief base accelerometer. */
    Vector3d base_attitude_;

    /** @brief base accelerometer. */
    Vector3d base_linear_acceleration_;

    /** @brief base attitude quaternion. */
    Vector4d base_attitude_quaternion_;

    /** @brief bias yaw. */
    double bias_yaw;

    /** @brief is it first loop. */
    bool first;

    /** @brief Integers from the serial port.
     * - 4 sliders in [0, 1024]
     * - emergency stop
     */
    std::vector<int> slider_box_data_;

    /**
     * Drivers communication objects
     */

    /**
     * @brief Main board drivers.
     *
     * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
     */
    std::shared_ptr<MasterBoardInterface> main_board_ptr_;

    /** @brief Main board blmc_drivers overlay.
     *
     * This object contains the API compatible with the blmc_drivers and
     * BLMCJointModule(s).
     */
    std::shared_ptr<blmc_drivers::SpiBus> spi_bus_;

    /** @brief These are the BOLT_NB_MOTOR_BOARD motor boards of the robot. */
    std::array<std::shared_ptr<blmc_drivers::SpiMotorBoard>,
               BOLT_NB_MOTOR_BOARD>
        motor_boards_;

    /** @brief motors_ are the objects allowing us to send motor commands and
     * receive data. */
    std::array<blmc_robots::MotorInterface_ptr, BOLT_NB_MOTOR> motors_;

    /** @brief Joint modules containing the driving system paramters */
    blmc_drivers::BlmcJointModules<BOLT_NB_MOTOR> joints_;

    /** @brief Address the rotation direction of the motor. */
    std::array<bool, BOLT_NB_MOTOR> reverse_polarities_;

    /**
     * @brief Reader for serial port to read arduino slider values.
     */
    std::shared_ptr<blmc_drivers::SerialReader> serial_reader_;

    bool safe_mode_ = false;
};

}  // namespace bolt
