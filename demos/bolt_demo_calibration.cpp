/**
 * @file demo_solo12_calibration.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Small demo to test the calibration on the real robot.
 * @version 0.1
 * @date 2019-11-08
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "bolt/bolt.hpp"
#include "bolt/utils.hpp"

using namespace bolt;
typedef ThreadCalibrationData<Bolt> ThreadCalibrationData_t;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr)
{
    ThreadCalibrationData_t* thread_data_ptr =
        (static_cast<ThreadCalibrationData_t*>(thread_data_void_ptr));

    Bolt& robot = *thread_data_ptr->robot;

    robot.wait_until_ready();

    // ask for calibration
    robot.request_calibration();

    // The calibration commend is computed in the send_target_joint_torque.
    Eigen::Vector6d dummy_command = Eigen::Vector6d::Zero();
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (!CTRL_C_DETECTED && robot.is_calibrating())
    {
        robot.acquire_sensors();
        robot.send_target_joint_torque(dummy_command);
        spinner.spin();
    }

    CTRL_C_DETECTED = true;
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    enable_ctrl_c();

    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_bolt_calibration "
            "network_id`.");
    }

    std::shared_ptr<Bolt> robot = std::make_shared<Bolt>();
    robot->initialize(argv[1]);

    ThreadCalibrationData_t thread_data;
    thread_data.robot = robot;

    rt_printf("Controller is set up.\n");
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(&control_loop, &thread_data);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
