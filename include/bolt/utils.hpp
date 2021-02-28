/**
 * @file common_demo_header.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Contains some default tools for creating demos
 * @version 0.1
 * @date 2019-11-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <Eigen/Eigen>
#include <iostream>

// manage the ctrl+c signal
#include <signal.h>
// thread safe flag for application shutdown management
#include <atomic>

#include "yaml_utils/yaml_cpp_fwd.hpp"
#include "real_time_tools/timer.hpp"
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/thread.hpp"

namespace bolt
{
/**
 * @brief This small structure is used for reading the calibration parameters
 * for the calibrations demos.
 *
 * @tparam ROBOT_TYPE
 */
template <class ROBOT_TYPE>
struct ThreadCalibrationData
{
    std::shared_ptr<ROBOT_TYPE> robot;
    Eigen::VectorXd joint_index_to_zero;

    ThreadCalibrationData(std::shared_ptr<ROBOT_TYPE> robot_in)
        : robot(robot_in)
    {
        std::string yaml_path = ODRI_CONTROL_INTERFACE_YAML_PATH;

        std::cout << "Loading paramters from " << yaml_path << std::endl;
        YAML::Node param = YAML::LoadFile(yaml_path);
        YAML::ReadParameter(param["joint_calibrator"],
                            "position_offsets",
                            joint_index_to_zero);
    }
};

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the
 * terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int)
{
    CTRL_C_DETECTED = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c()
{
    // make sure we catch the ctrl+c signal to kill the application
    // properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    CTRL_C_DETECTED = false;
}

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name, Eigen::Ref<Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%0.3f, ", v(i));
    }
    rt_printf("]\n");
}

}  // namespace bolt