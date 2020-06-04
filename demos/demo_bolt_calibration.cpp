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
#include "bolt/common_demo_header.hpp"

using namespace bolt;
typedef ThreadCalibrationData<Bolt> ThreadCalibrationData_t;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr)
{
    ThreadCalibrationData_t* thread_data_ptr =
        (static_cast<ThreadCalibrationData_t*>(thread_data_void_ptr));
    Vector6d joint_index_to_zero = thread_data_ptr->joint_index_to_zero;
    thread_data_ptr->robot->calibrate(joint_index_to_zero);

    blmc_robots::CTRL_C_DETECTED = true;
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    blmc_robots::enable_ctrl_c();

    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_solo12_calibration "
            "network_id`.");
    }

    std::shared_ptr<Bolt> robot = std::make_shared<Bolt>();
    robot->initialize(argv[1]);

    ThreadCalibrationData_t thread_data(robot);

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
