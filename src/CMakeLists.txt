set(all_src_targets)

#
# Declare the project library.
#

add_library(bolt SHARED bolt.cpp)
# Add includes
target_include_directories(
  bolt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(bolt blmc_drivers::blmc_drivers)
target_link_libraries(bolt odri_control_interface::odri_control_interface)
target_link_libraries(bolt real_time_tools::real_time_tools)
target_link_libraries(bolt yaml_utils::yaml_utils)
target_link_libraries(bolt Eigen3::Eigen)
# YAML parameters.
string(
  CONCAT odri_control_interface_yaml_path
         "${PythonModules_robot_properties_bolt_PATH}/"
         "robot_properties_bolt/robot_properties_bolt/odri_control_interface/"
         "bolt_driver.yaml")
target_compile_definitions(
  bolt
  PUBLIC ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
# Export the target.
list(APPEND all_src_targets bolt)

#
# Hardware calibration program.
#
add_executable(bolt_hardware_calibration programs/hardware_calibration.cpp)
# Add the include dependencies.
target_include_directories(
  bolt_hardware_calibration
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(bolt_hardware_calibration bolt)
# Export the target.
list(APPEND all_src_targets bolt_hardware_calibration)

#
# Optionally compile the dynamic_graph_manager Bolt wrapper.
#
if(${dynamic_graph_manager_FOUND})
  #
  # create library
  #
  add_library(dgm_bolt SHARED dgm_bolt.cpp)
  # Add the include dependencies.
  target_include_directories(
    dgm_bolt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dgm_bolt ${PROJECT_NAME})
  target_link_libraries(dgm_bolt dynamic_graph_manager::dynamic_graph_manager)
  # Export the target.
  list(APPEND all_src_targets dgm_bolt)

  #
  # Create the robot executable.
  #
  add_executable(dg_main_bolt programs/dg_main_bolt.cpp)
  # Add the include dependencies.
  target_include_directories(
    dg_main_bolt
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dg_main_bolt dgm_bolt)
  # YAML parameters.
  string(
    CONCAT dynamic_graph_manager_yaml_file
           "${PythonModules_robot_properties_bolt_PATH}/"
           "robot_properties_bolt/robot_properties_bolt/dynamic_graph_manager/"
           "dgm_parameters_bolt.yaml")
  target_compile_definitions(
    dg_main_bolt
    PUBLIC
      DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
      ODRI_CONTROL_INTERFACE_YAML_PATH="${ODRI_CONTROL_INTERFACE_YAML_PATH}")
  # Export.
  list(APPEND all_src_targets dg_main_bolt)

endif(${dynamic_graph_manager_FOUND})

#
# Install and Export the libraries an programs.
#
install(
  TARGETS ${all_src_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
