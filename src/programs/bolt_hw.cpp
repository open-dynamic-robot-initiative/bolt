/**
 * \file bolt.cpp
 * \brief Execute the main program to control the bolt
 * \author Maximilien Naveau
 * \date 2018
 *
 * DynamicGraphManager for bolt main executable.
 */

#include <fstream>
#include "bolt/dgm_bolt.hpp"

int main(int, char*[])
{
    // Bolt ready
    shared_memory::clear_shared_memory("bolt");
    shared_memory::set("bolt", "ready", false);
    // ShM name
    shared_memory::clear_shared_memory("dgm_shm_name");
    shared_memory::set<std::string>(
        "dgm_shm_name", "shared_memory_name", dynamic_graph_manager::DynamicGraphManager::shared_memory_name_);
    // clean the shared memory
    shared_memory::clear_shared_memory(dynamic_graph_manager::DynamicGraphManager::shared_memory_name_);
    // we create and destroy the condition variable to free the shared memory
    // and therefore the associated mutex which must be lockable at this state.
    {
        shared_memory::LockedConditionVariable(dynamic_graph_manager::DynamicGraphManager::cond_var_name_, true);
    }
    shared_memory::set("bolt", "ready", true);

    // Get the dynamic_graph_manager config file.
    std::string yaml_path = DYNAMIC_GRAPH_MANAGER_YAML_PATH;

    std::cout << "Loading parameters from " << yaml_path << std::endl;
    
    std::ifstream f(yaml_path.c_str());
    if (!f.good())
    {
        throw std::runtime_error("Error: " + yaml_path + " not found!");
    }
    YAML::Node param = YAML::LoadFile(yaml_path);
    // Create the dgm.
    bolt::DGMBolt dgm;

    // Initialize and run it.
    dgm.initialize(param);

    dgm.initialize_hardware_communication_process();
    dgm.run_hardware_communication_process();

    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}