set(all_lib_targets)
set(all_exe_targets)

#
# Declare the project library.
#

add_library(bolt SHARED bolt.cpp bolt_humanoid.cpp)
# Add includes
target_include_directories(
  bolt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(bolt odri_control_interface::odri_control_interface)
target_link_libraries(bolt slider_box::slider_box)
target_link_libraries(bolt real_time_tools::real_time_tools)
target_link_libraries(bolt yaml_utils::yaml_utils)
target_link_libraries(bolt Eigen3::Eigen)
# YAML parameters.
string(
  CONCAT odri_control_interface_yaml_path
         "${PythonModules_robot_properties_bolt_PATH}/"
         "robot_properties_bolt/resources/odri_control_interface/"
         "bolt_driver.yaml")
target_compile_definitions(
  bolt
  PUBLIC ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")

string(
  CONCAT odri_control_interface_humanoid_yaml_path
          "${PythonModules_robot_properties_bolt_PATH}/"
          "robot_properties_bolt/resources/odri_control_interface/"
          "bolt_humanoid_driver.yaml")
target_compile_definitions(
  bolt
  PUBLIC ODRI_CONTROL_INTERFACE_HUMANOID_YAML_PATH="${odri_control_interface_humanoid_yaml_path}")

# Export the target.
list(APPEND all_lib_targets bolt)

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
list(APPEND all_exe_targets bolt_hardware_calibration)

add_executable(bolt_humanoid_hardware_calibration programs/hardware_humanoid_calibration.cpp)
# Add the include dependencies.
target_include_directories(
  bolt_humanoid_hardware_calibration
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(bolt_humanoid_hardware_calibration bolt)
# Export the target.
list(APPEND all_exe_targets bolt_humanoid_hardware_calibration)

#
# Optionally compile the dynamic_graph_manager Bolt wrapper.
#
if(${dynamic_graph_manager_FOUND})
  #
  # create library
  #
  add_library(dgm_bolt SHARED dgm_bolt.cpp dgm_bolt_humanoid.cpp)
  # Add the include dependencies.
  target_include_directories(
    dgm_bolt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dgm_bolt bolt)
  target_link_libraries(dgm_bolt dynamic_graph_manager::dynamic_graph_manager)
  # Export the target.
  list(APPEND all_lib_targets dgm_bolt)

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
           "robot_properties_bolt/resources/dynamic_graph_manager/"
           "dgm_parameters_bolt.yaml")
  target_compile_definitions(
    dg_main_bolt
    PUBLIC DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
           ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
  # Export.
  list(APPEND all_exe_targets dg_main_bolt)

  #
  # Create the robot executable.
  #
  add_executable(dg_main_bolt_humanoid programs/dg_main_bolt_humanoid.cpp)
  # Add the include dependencies.
  target_include_directories(
    dg_main_bolt_humanoid
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dg_main_bolt_humanoid dgm_bolt)
  # YAML parameters.
  string(
    CONCAT dynamic_graph_manager_humanoid_yaml_file
           "${PythonModules_robot_properties_bolt_PATH}/"
           "robot_properties_bolt/resources/dynamic_graph_manager/"
           "dgm_parameters_bolt_humanoid.yaml")
  target_compile_definitions(
    dg_main_bolt_humanoid
    PUBLIC DYNAMIC_GRAPH_MANAGER_HUMANOID_YAML_PATH="${dynamic_graph_manager_humanoid_yaml_file}"
           ODRI_CONTROL_INTERFACE_HUMANOID_YAML_PATH="${odri_control_interface_humanoid_yaml_path}")
  # Export.
  list(APPEND all_exe_targets dg_main_bolt_humanoid)

  #
  # Create the robot executable.
  #
  add_executable(bolt_dg programs/bolt_dg.cpp)
  # Add the include dependencies.
  target_include_directories(
    bolt_dg
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(bolt_dg dgm_bolt)
  # YAML parameters.
  string(
    CONCAT dynamic_graph_manager_yaml_file
           "${PythonModules_robot_properties_bolt_PATH}/"
           "robot_properties_bolt/resources/dynamic_graph_manager/"
           "dgm_parameters_bolt.yaml")
  target_compile_definitions(
    bolt_dg
    PUBLIC DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
           ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
  # Export.
  list(APPEND all_exe_targets bolt_dg)

  #
  # Create the robot executable.
  #
  add_executable(bolt_hw programs/bolt_hw.cpp)
  # Add the include dependencies.
  target_include_directories(
          bolt_hw
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(bolt_hw dgm_bolt)
  # YAML parameters.
  string(
    CONCAT dynamic_graph_manager_yaml_file
           "${PythonModules_robot_properties_bolt_PATH}/"
           "robot_properties_bolt/resources/dynamic_graph_manager/"
           "dgm_parameters_bolt.yaml")
  target_compile_definitions(
          bolt_hw
    PUBLIC DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
           ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
  # Export.
  list(APPEND all_exe_targets bolt_hw)

endif(${dynamic_graph_manager_FOUND})

#
# Install and Export the libraries an programs.
#
install(
  TARGETS ${all_lib_targets} ${all_exe_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

foreach(target ${all_exe_targets})
  # install a symlink of the executable in lib/bolt
  string(
    CONCAT symlink_command
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E make_directory "
           "  ${CMAKE_INSTALL_PREFIX}/bin/)\n"
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E create_symlink "
           "  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${target}"
           "  ${CMAKE_INSTALL_PREFIX}/bin/${target} )"
  )
  install(CODE ${symlink_command})
endforeach()
