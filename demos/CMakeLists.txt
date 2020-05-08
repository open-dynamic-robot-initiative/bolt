#####################
# create some demos #
#####################


include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_demo demo_name)
  # Cannot use ARGN directly with list() command,
  # so copy it to a variable first.
  set (extra_macro_args ${ARGN})

  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
      list(GET extra_macro_args 0 config_file)
  endif ()
  
  # Add the executable of the demo.
  add_executable(${demo_name}
    ${demo_name}.cpp
  )
  
  # Add the dependencies.
  target_link_libraries(${demo_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
  )

  # Check if the file exists.
  if(config_file)
    if(NOT EXISTS ${config_file})
      message(FATAL_ERROR "${demo_name}: Error cannot find ${config_file}")
    endif()
    target_compile_definitions(${demo_name} PUBLIC
      YAML_PARAMS="${config_file}")
  endif()

endmacro(create_demo demo_name)

###############
# build demos #
###############

create_demo(demo_bolt)
create_demo(demo_bolt_calibration
    "${robot_properties_bolt_SOURCE_PREFIX}/config/dgm_parameters_bolt.yaml")
