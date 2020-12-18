#
# Declare a simple macro to build the demos.
#

set(all_demo_targets)

macro(create_demo demo_name)
  # Cannot use ARGN directly with list() command,
  # so copy it to a variable
  set(extra_macro_args ${ARGN})

  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
      list(GET extra_macro_args 0 config_file)
  endif ()
  
  # Add the executable of the demo.
  add_executable(${demo_name}
    ${demo_name}.cpp
  )


  target_include_directories(${demo_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  
  # Add the dependencies.
  target_link_libraries(${demo_name} ${PROJECT_NAME})

  # Check if the file exists.
  if(package_name AND resource_path)
    target_compile_definitions(
    demo_${demo_name} PUBLIC
    ROBOT_PROPERTIES_PACKAGE_NAME="${package_name}"
    ROBOT_PROPERTIES_YAML_PATH="${config_file}")
  endif()

endmacro(create_demo demo_name)

#
# Build demos.
#

create_demo(demo_bolt)


create_demo(demo_bolt_calibration robot_properties_bolt dgm_parameters_bolt.yaml)
