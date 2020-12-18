#
# Python bindings.
#

##
add_library(py_bolt MODULE py_bolt.cpp)

target_link_libraries(py_bolt PRIVATE pybind11::module)

target_link_libraries(py_bolt PRIVATE bolt)

set_target_properties(py_bolt PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_include_directories(py_bolt PUBLIC
    BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> SYSTEM
    PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS py_bolt DESTINATION ${PYTHON_INSTALL_DIR})
