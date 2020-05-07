######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(bolt)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  real_time_tools
  pybind11_catkin
  yaml_cpp_catkin
  
  blmc_drivers
  blmc_robots
  robot_properties_bolt
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_eigen()

##################
# Python package #
##################

catkin_python_setup()

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

##########################################
# export the package as a catkin package #
##########################################
set(exported_libraries
  ${PROJECT_NAME}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${exported_libraries}
  CATKIN_DEPENDS ${CATKIN_PKGS}
)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

####################
# manage the demos #
####################
add_subdirectory(demos)

###################
# python bindings #
###################
add_subdirectory(srcpy)

##########################
# building documentation #
##########################
build_doxygen_documentation()
