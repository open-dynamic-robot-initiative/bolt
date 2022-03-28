/**
 * \file bolt.cpp
 * \brief Execute the main program to control the bolt
 * \author Maximilien Naveau
 * \date 2018
 *
 * DynamicGraphManager for bolt main executable.
 */

#include <fstream>
#include "bolt/dgm_bolt_humanoid.hpp"

int main(int, char*[])
{
    // Get the dynamic_graph_manager config file.
    std::string yaml_path = DYNAMIC_GRAPH_MANAGER_HUMANOID_YAML_PATH;

    std::cout << "Loading parameters from " << yaml_path << std::endl;

    std::ifstream f(yaml_path.c_str());
    if (!f.good())
    {
        throw std::runtime_error("Error: " + yaml_path + " not found!");
    }
    YAML::Node param = YAML::LoadFile(yaml_path);
    // Create the dgm.
    bolt::DGMBoltHumanoid dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();
//    dgm.run_single_process();
}