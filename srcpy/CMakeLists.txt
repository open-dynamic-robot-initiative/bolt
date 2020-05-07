###################
# python bindings #
###################

##
set(py_bolt_SRC_FILES
  py_bolt.cpp
)
pybind11_add_module(py_bolt ${py_bolt_SRC_FILES})
target_link_libraries(py_bolt PRIVATE bolt)
set_target_properties(py_bolt PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    LIBRARY_OUTPUT_NAME bolt
    PREFIX "")
install(TARGETS py_bolt DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
