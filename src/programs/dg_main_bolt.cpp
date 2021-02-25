/**
 * \file bolt.cpp
 * \brief Execute the main program to control the bolt
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include "bolt/dgm_bolt.hpp"

int main(int, char*[])
{
    // Get the dynamic_graph_manager config file.
    std::cout << "Loading parameters from " << DYNAMIC_GRAPH_MANAGER_YAML_PATH
              << std::endl;
    std::string yaml_path = DYNAMIC_GRAPH_MANAGER_YAML_PATH;
    YAML::Node param = YAML::LoadFile(yaml_path);
    // Create the dgm.
    bolt::DGMBolt dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();
}