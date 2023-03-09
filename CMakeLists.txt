#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft. License
# BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(bolt VERSION 1.0.0)

# specify the C++ 17 standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# external dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11 REQUIRED)

# local dependencies.
find_package(ament_cmake REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(slider_box REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(PythonModules REQUIRED COMPONENTS robot_properties_bolt)

# optional dependencies.
find_package(dynamic_graph_manager QUIET)

# Export dependencies.
ament_export_dependencies(
  odri_control_interface
  real_time_tools
  yaml_utils
  slider_box
)
if(${dynamic_graph_manager_FOUND})
  ament_export_dependencies(dynamic_graph_manager)
endif()

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

# Install includes.
install(DIRECTORY include/ DESTINATION include)

# Install python files.
get_python_install_dir(python_install_dir)
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# Building documentation.
#
add_documentation()

#
# Export this package as a cmake package.
#
ament_package()
