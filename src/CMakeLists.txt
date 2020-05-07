###############################
# Declare the project library #
###############################

add_library(${PROJECT_NAME} SHARED
  bolt.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${PROJECT_NAME} rt pthread)
endif()

set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${PROJECT_NAME})

################################
# Hardware calibration program #
################################

add_executable(hardware_calibration
  programs/hardware_calibration.cpp
)
target_link_libraries(hardware_calibration
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
