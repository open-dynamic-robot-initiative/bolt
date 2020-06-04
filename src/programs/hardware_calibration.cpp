/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief This program is used to perform the hardware calibration of the robot.
 * Procedure:
 *  - You must start the robot in a position that is close to the "zero" pose.
 *  - Then start the demo: roscd && sudo ./lib/bolt/hardware_calibration
 * <network_id> The "network_id" is the left column of the output of `ifconfig`
 * in bash.
 *  - The robot is going to find the closest index in a specific direction and
 *      go idle (0 torques sent).
 *  - In the terminal you have displayed the distance between the zero and the
 *      found index. Therefore once you placed the joints to the proper zero
 *      configuration you can just kill the application with ctrl+c and copy the
 *      date to you favorite configuration file.
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
