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
    sleep(2);
    bool ready = false;
    while(!ready)
    {
        try{
            shared_memory::get("bolt", "ready", ready);
        }catch(...)
        {
            ready = false;
        }
        sleep(0.5);
        std::cout << "Wait for the shared memory to be ready." << std::endl;
    }
    std::cout << "Shared memory ready." << std::endl;

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

    dgm.initialize_dynamic_graph_process();
    dgm.run_dynamic_graph_process();
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
    std::cout << "DG: End of the dynamic graph process." << std::endl;
}