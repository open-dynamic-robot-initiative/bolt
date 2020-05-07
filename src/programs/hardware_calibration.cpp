/**
 * \file solo12_hardware_calibration.cpp
 * \brief ...
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Bolt class in a small demo.
 */

#include "blmc_robots/common_programs_header.hpp"
#include "bolt/bolt.hpp"

using namespace blmc_robots;
using namespace bolt;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Bolt& robot = *(static_cast<Bolt*>(robot_void_ptr));

    Vector6d joint_index_to_zero;
    joint_index_to_zero.fill(0.0);
    bool good_calibration = robot.calibrate(joint_index_to_zero);

    long int count = 0;
    while (!CTRL_C_DETECTED && good_calibration)
    {
        if (count % 200 == 0)
        {
            robot.acquire_sensors();
            print_vector("Joint Positions", robot.get_joint_positions());
        }
    }

    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `sudo ./hardware_calibration "
            "network_id`.");
    }

    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Bolt robot;

    robot.initialize(argv[1]);

    rt_printf("Controller is set up.\n");
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
