/**
 * \file demo_bolt.cpp
 * \brief Implements basic PD controller reading slider values.
 * \author Julian Viereck
 * \date 21 November 2019
 *
 * This file uses the Solo12 class in a small demo.
 */

#include "bolt/bolt.hpp"
#include "bolt/utils.hpp"

using namespace bolt;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* args)
{
    Bolt& robot = *static_cast<Bolt*>(args);

    // 0 torques
    Eigen::Vector6d desired_torque = Eigen::Vector6d::Zero();
    rt_printf("Sensor reading loop started \n");

    robot.wait_until_ready();

    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            rt_printf("Sensory data:");
            rt_printf("\n");
            print_vector("des joint_tau                  ",
                         desired_torque);
            print_vector("act joint_pos                  ",
                         robot.get_joint_positions());
            print_vector("act joint_vel                  ",
                         robot.get_joint_velocities());
            print_vector("act joint torq                 ",
                         robot.get_joint_target_torques());
            print_vector("act joint target torq          ",
                         robot.get_joint_torques());
            print_vector_bool("act status motor ready         ",
                         robot.get_motor_ready());
            print_vector_bool("act status motor enabled       ",
                         robot.get_motor_enabled());
            print_vector_bool("act status motor board enabled ",
                         robot.get_motor_board_enabled());
            print_vector_int("act status motor board errors  ",
                         robot.get_motor_board_errors());
            print_vector("act slider pos                 ",
                         robot.get_slider_positions());
            print_vector("act imu quat                   ",
                         robot.get_base_attitude());
            print_vector("act imu acc                    ",
                         robot.get_base_accelerometer());
            print_vector("act imu gyroscope              ",
                         robot.get_base_gyroscope());
            print_vector("act imu lin acc                ",
                         robot.get_base_linear_acceleration());
            rt_printf("act e-stop                     : %s\n",
                      robot.get_active_estop() ? "true" : "false");
            rt_printf("has error                      : %s\n",
                      robot.has_error() ? "true" : "false");
            rt_printf("\n");
            fflush(stdout);
        }
        ++count;

        // Send the current to the motor
        // desired_torque.setZero();
        robot.send_target_joint_torque(desired_torque);

        real_time_tools::Timer::sleep_sec(0.001);
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_bolt network_id`.");
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    Bolt robot;
    robot.initialize(std::string(argv[1]));

    rt_printf("sensor reader is set up \n");
    thread.create_realtime_thread(&control_loop, &robot);
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
