#
# Declare a simple macro to build the demos.
#

set(all_demo_targets)

string(
  CONCAT odri_control_interface_yaml_path
         "${PythonModules_robot_properties_bolt_PATH}/"
         "robot_properties_bolt/robot_properties_bolt/odri_control_interface/"
         "bolt_driver.yaml")

macro(create_demo demo_name)
  # Add the executable of the demo.
  add_executable(${demo_name} ${demo_name}.cpp)

  target_include_directories(
    ${demo_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

  # Add the dependencies.
  target_link_libraries(${demo_name} ${PROJECT_NAME})

  # Check if the file exists. if(package_name AND resource_path)
  target_compile_definitions(
    ${demo_name}
    PUBLIC
      ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")

  # Install demo
  install(
    TARGETS ${demo_name}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include)

  # Install a symlink of the executable in lib/bolt
  string(
    CONCAT symlink_command
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E make_directory "
           "  ${CMAKE_INSTALL_PREFIX}/bin/)\n"
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E create_symlink "
           "  ${CMAKE_INSTALL_PREFIX}/bin/${demo_name}"
           "  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${demo_name})"
  )
  install(CODE ${symlink_command})


endmacro(create_demo demo_name)

#
# Build demos.
#

create_demo(bolt_demo_pd)

create_demo(bolt_demo_calibration)

create_demo(bolt_demo_sensor_reading)
