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

#include "blmc_robots/common_programs_header.hpp"

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
        std::cout << "Loading paramters from " << YAML_PARAMS << std::endl;
        YAML::Node param = YAML::LoadFile(YAML_PARAMS);
        YAML::ReadParameter(param["hardware_communication"]["calibration"],
                            "index_to_zero_angle",
                            joint_index_to_zero);
    }
};

}  // namespace bolt