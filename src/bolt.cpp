#include <cmath>

#include "bolt/bolt.hpp"

namespace bolt
{
const double Bolt::max_joint_torque_security_margin_ = 0.99;

Bolt::Bolt()
{
    /**
     * Hardware properties
     */
    motor_inertias_.setZero();
    motor_torque_constants_.setZero();
    joint_gear_ratios_.setZero();
    motor_max_current_.setZero();
    max_joint_torques_.setZero();
    joint_zero_positions_.setZero();
    reverse_polarities_.fill(false);

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
    joint_encoder_index_.setZero();

    /**
     * Additional data
     */
    map_joint_id_to_motor_board_id_.fill(0);
    map_joint_id_to_motor_port_id_.fill(0);
    network_id_ = "";
    map_joint_id_to_motor_port_id_.fill(0);
    slider_positions_.setZero();
    // By default assume the estop is active.
    active_estop_ = true;
    // 4 sliders + 1 emergency stop button.
    slider_box_data_.resize(BOLT_NB_SLIDER + 1, 0);
    base_accelerometer_.setZero();
    base_gyroscope_.setZero();
    base_attitude_.setZero();
    base_linear_acceleration_.setZero();
    base_attitude_quaternion_.setZero();

    /**
     * Setup some known data
     */

    // for now this value is very small but it is currently for debug mode
    motor_max_current_.fill(12.0);  // TODO: set as paramters?
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);
    first = true;
}

void Bolt::initialize(const std::string& network_id)
{
    network_id_ = network_id;

    // Create the different mapping
    map_joint_id_to_motor_board_id_ = {0, 1, 1, 0, 2, 2};
    map_joint_id_to_motor_port_id_ = {0, 1, 0, 1, 1, 0};

    // Initialize the communication with the main board.
    main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id_);
    main_board_ptr_->Init();

    // create the SpiBus (blmc_drivers wrapper around the MasterBoardInterface)
    spi_bus_ = std::make_shared<blmc_drivers::SpiBus>(main_board_ptr_,
                                                      motor_boards_.size());

    // create the motor board objects:
    for (size_t mb_id = 0; mb_id < motor_boards_.size(); ++mb_id)
    {
        motor_boards_[mb_id] =
            std::make_shared<blmc_drivers::SpiMotorBoard>(spi_bus_, mb_id);
    }

    // Create the motors object. j_id is the joint_id
    for (unsigned j_id = 0; j_id < motors_.size(); ++j_id)
    {
        motors_[j_id] = std::make_shared<blmc_drivers::Motor>(
            motor_boards_[map_joint_id_to_motor_board_id_[j_id]],
            map_joint_id_to_motor_port_id_[j_id]);
    }

    // Create the joint module objects
    joints_.set_motor_array(motors_,
                            motor_torque_constants_,
                            joint_gear_ratios_,
                            joint_zero_positions_,
                            motor_max_current_);

    // Set the maximum joint torque available
    max_joint_torques_ =
        max_joint_torque_security_margin_ * joints_.get_max_torques().array();

    // fix the polarity to be the same as the urdf model.
    reverse_polarities_ = {true, true, true, true, false, false};
    joints_.set_joint_polarities(reverse_polarities_);

    // The the control gains in order to perform the calibration
    blmc_robots::Vector6d kp, kd;
    kp.fill(2.0);
    kd.fill(0.05);
    joints_.set_position_control_gains(kp, kd);

    // Use a serial port to read slider values.
    serial_reader_ = std::make_shared<blmc_drivers::SerialReader>(
        "serial_port", BOLT_NB_SLIDER + 1);

    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    while (!main_board_ptr_->IsTimeout() && !main_board_ptr_->IsAckMsgReceived()) {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
        {
            last = std::chrono::system_clock::now();
            main_board_ptr_->SendInit();
        }
    }

    // Wait until all the motors are ready.
    spi_bus_->wait_until_ready();

    rt_printf("All motors and boards are ready.\n");
}


void Bolt::fill_base_attitude_quaternion(){
    double sr = sin(base_attitude_[0]/2.);
    double cr = cos(base_attitude_[0]/2.);
    double sp = sin(base_attitude_[1]/2.);
    double cp = cos(base_attitude_[1]/2.);
    double sy = sin(base_attitude_[2]/2.);
    double cy = cos(base_attitude_[2]/2.);

    base_attitude_quaternion_ << sr * cp * cy - cr * sp * sy,
                                 cr * sp * cy + sr * cp * sy,
                                 cr * cp * sy - sr * sp * cy,
                                 cr * cp * cy + sr * sp * sy;
}

void Bolt::acquire_sensors() {
    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = joints_.get_measured_angles();
    // acquire the joint velocities
    joint_velocities_ = joints_.get_measured_velocities();
    // acquire the joint torques
    joint_torques_ = joints_.get_measured_torques();
    // acquire the target joint torques
    joint_target_torques_ = joints_.get_sent_torques();

    // The index angle is not transmitted.
    joint_encoder_index_ = joints_.get_measured_index_angles();

    /**
     * Additional data
     */
    base_accelerometer_ << main_board_ptr_->imu_data_accelerometer(0),
            main_board_ptr_->imu_data_accelerometer(1),
            main_board_ptr_->imu_data_accelerometer(2);

//    base_gyroscope_ << main_board_ptr_->imu_data_gyroscope(2),
//                       -main_board_ptr_->imu_data_gyroscope(0),
//                       main_board_ptr_->imu_data_gyroscope(1);

    base_gyroscope_ << -main_board_ptr_->imu_data_gyroscope(0),
            -main_board_ptr_->imu_data_gyroscope(1),
            -main_board_ptr_->imu_data_gyroscope(2);

//    base_attitude_ << main_board_ptr_->imu_data_attitude(1),
//                      main_board_ptr_->imu_data_attitude(0) - M_PI / 2,
//                      -main_board_ptr_->imu_data_attitude(2);

    if (first == true){
        bias_yaw = main_board_ptr_->imu_data_attitude(2);
        first = false;
    }

    base_attitude_ << main_board_ptr_->imu_data_attitude(0),
            main_board_ptr_->imu_data_attitude(1),
            main_board_ptr_->imu_data_attitude(2) - bias_yaw;


    base_linear_acceleration_ << main_board_ptr_->imu_data_linear_acceleration(0),
                                 main_board_ptr_->imu_data_linear_acceleration(1),
                                 main_board_ptr_->imu_data_linear_acceleration(2);

    fill_base_attitude_quaternion();

    // acquire the slider positions
    /// @todo: Handle case that no new values are arriving.
    serial_reader_->fill_vector(slider_box_data_);
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
    for (size_t i = 0; i < motor_boards_.size(); ++i)
    {
        const blmc_drivers::MotorBoardStatus& motor_board_status =
            motor_boards_[i]->get_status()->newest_element();
        motor_board_enabled_[i] = motor_board_status.system_enabled;
        motor_board_errors_[i] = motor_board_status.error_code;
    }
    // motors status
    for (size_t j_id = 0; j_id < motors_.size(); ++j_id)
    {
        const blmc_drivers::MotorBoardStatus& motor_board_status =
            motor_boards_[map_joint_id_to_motor_board_id_[j_id]]
                ->get_status()
                ->newest_element();

        motor_enabled_[j_id] = (map_joint_id_to_motor_port_id_[j_id] == 1)
                                   ? motor_board_status.motor2_enabled
                                   : motor_board_status.motor1_enabled;
        motor_ready_[j_id] = (map_joint_id_to_motor_port_id_[j_id] == 1)
                                 ? motor_board_status.motor2_ready
                                 : motor_board_status.motor1_ready;
    }
}

void Bolt::send_target_joint_torque(
    const Eigen::Ref<Vector6d> target_joint_torque)
{
    Vector6d ctrl_torque;
    if(base_attitude_(1) < 0.6 && base_attitude_(1) > -0.6 &&
       (base_attitude_(0) < -2.5 || base_attitude_(0) > 2.5) && !safe_mode_) {
        ctrl_torque= target_joint_torque;
    }
    else if(!safe_mode_){
        safe_mode_ = true;
        std::cout << "Attitude Safe Mode zero torque\n";
        ctrl_torque = 0. * target_joint_torque;
    }
    else{
        ctrl_torque = 0. * target_joint_torque;
    }
    ctrl_torque = ctrl_torque.array().min(max_joint_torques_);
    ctrl_torque = ctrl_torque.array().max(-max_joint_torques_);
    joints_.set_torques(ctrl_torque);
    joints_.send_torques();
}

bool Bolt::calibrate(const Vector6d& home_offset_rad)
{
    // Maximum distance is twice the angle between joint indexes
    double search_distance_limit_rad =
        10.0 * (2.0 * M_PI / joint_gear_ratios_(0));
    Vector6d profile_step_size_rad = Vector6d::Constant(0.001);
    blmc_robots::HomingReturnCode homing_return_code = joints_.execute_homing(
        search_distance_limit_rad, home_offset_rad, profile_step_size_rad);
    if (homing_return_code == blmc_robots::HomingReturnCode::FAILED)
    {
        return false;
    }
    Vector6d zero_pose = Vector6d::Zero();
    joints_.go_to(zero_pose);
    return true;
}

}  // namespace bolt
