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

    // Using conversion from PD gains from example.cpp
    double kp = 3.0 * 9 * 0.025;
    double kd = 0.1 * 9 * 0.025;
    double t = 0.0;
    double dt = 0.001;
    double freq = 0.3;
    double amplitude = M_PI / 8.0;
    Eigen::Vector6d desired_joint_position = Eigen::Vector6d::Zero();
    Eigen::Vector6d desired_torque = Eigen::Vector6d::Zero();

    Eigen::Vector6d init_pose;
    Eigen::Matrix<bool, 6, 1> motor_enabled;

    robot.acquire_sensors();
    Eigen::Vector6d initial_joint_positions = robot.get_joint_positions();

    rt_printf("control loop started \n");

    robot.wait_until_ready();

    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // acquire the motor enabled signal.
        motor_enabled = robot.get_motor_enabled();

        // Desired pose and vel
        desired_joint_position =
            initial_joint_positions +
            Eigen::Vector6d::Ones() * amplitude * sin(2 * M_PI * freq * t);
        t += dt;

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            print_vector("des joint_tau  : ", desired_torque);
            print_vector("des joint_pos  : ", desired_joint_position);
            rt_printf("\n");
            print_vector("act joint_pos  : ", robot.get_joint_positions());
            print_vector("act joint_vel  : ", robot.get_joint_velocities());
            print_vector("act slider pos : ", robot.get_slider_positions());
            rt_printf("act e-stop     : %s\n",
                      robot.get_active_estop() ? "true" : "false");

            fflush(stdout);
        }
        ++count;

        // Send the current to the motor
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

    rt_printf("controller is set up \n");
    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("control loop started \n");
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
