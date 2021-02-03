set(all_src_targets)

#
# Declare the project library.
#

add_library(${PROJECT_NAME} SHARED bolt.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

# Link the dependencies.
target_link_libraries(${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
target_link_libraries(${PROJECT_NAME} blmc_drivers::blmc_drivers)
target_link_libraries(${PROJECT_NAME} blmc_robots::blmc_robots)
target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
# target_link_libraries(${PROJECT_NAME} robot_interfaces::robot_interfaces)
target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Export the target.
list(APPEND all_src_targets ${PROJECT_NAME})

# add_library(bolt SHARED bolt.cpp) Add the include dependencies.
# target_include_directories( bolt PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
# Link the dependencies. target_link_libraries(bolt ${PROJECT_NAME}) Export the
# target. list(APPEND all_src_targets bolt)

add_executable(hardware_calibration programs/hardware_calibration.cpp)
# Add the include dependencies.
target_include_directories(
  hardware_calibration PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(hardware_calibration bolt)
# Export the target.
list(APPEND all_src_targets hardware_calibration)

# set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${PROJECT_NAME})

# ##############################################################################
# Hardware calibration program #
# ##############################################################################

# add_executable(hardware_calibration programs/hardware_calibration.cpp )

# target_link_libraries(hardware_calibration ${PROJECT_NAME} ${catkin_LIBRARIES}
# )

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
