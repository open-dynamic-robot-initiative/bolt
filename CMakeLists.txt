#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft. License
# BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(bolt VERSION 1.0.0)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC")

# build tools
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# depend on ament macros
find_package(ament_index_cpp REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(blmc_robots REQUIRED)

# export de dependencies
ament_export_dependencies(
  ament_cmake
  mpi_cmake_modules
  ament_index_cpp
  blmc_drivers
  real_time_tools
  pybind11
  yaml_utils
  blmc_robots)

# prepare the final export
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

#
# Manage the unit tests.
#
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#
# Manage the demos.
#
add_subdirectory(demos)

#
# Python bindings.
#
add_subdirectory(srcpy)

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
