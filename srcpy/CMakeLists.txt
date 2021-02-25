#
# Python bindings.
#

# Python wrapper
set(wrapper_name bolt_cpp)

add_library(${wrapper_name} MODULE py_bolt.cpp)

target_link_libraries(${wrapper_name} PRIVATE pybind11::module)

target_link_libraries(${wrapper_name} PRIVATE bolt)

set_target_properties(${wrapper_name} PROPERTIES PREFIX "" SUFFIX
                                                   "${PYTHON_MODULE_EXTENSION}")

target_include_directories(
  ${wrapper_name}
  PUBLIC BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})

get_python_install_dir(python_install_dir)

install(TARGETS ${wrapper_name} DESTINATION ${python_install_dir})
