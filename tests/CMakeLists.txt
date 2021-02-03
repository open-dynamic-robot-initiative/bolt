#
# Add unit tests.
#

# Use gtest
find_package(ament_cmake_gtest)

# Create a usefull macro to build different unit tests suits.
macro(create_blmc_robots_unittest test_name)

  # create the executable
  ament_add_gtest(${test_name}_ut main.cpp test_${test_name}.cpp)
  # link the dependecies to it
  target_link_libraries(${test_name}_ut ${PROJECT_NAME})
  # add some preprocessor variable
  target_compile_definitions(
    ${test_name}_ut
    PUBLIC TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/")

endmacro(create_blmc_robots_unittest test_name)
