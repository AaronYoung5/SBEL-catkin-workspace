# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "teleop_test: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iteleop_test:/home/aaron/ROS/catkin/src/teleop_test/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(teleop_test_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_custom_target(_teleop_test_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teleop_test" "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(teleop_test
  "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_test
)

### Generating Services

### Generating Module File
_generate_module_cpp(teleop_test
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_test
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(teleop_test_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(teleop_test_generate_messages teleop_test_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_dependencies(teleop_test_generate_messages_cpp _teleop_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_test_gencpp)
add_dependencies(teleop_test_gencpp teleop_test_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_test_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(teleop_test
  "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_test
)

### Generating Services

### Generating Module File
_generate_module_eus(teleop_test
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_test
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(teleop_test_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(teleop_test_generate_messages teleop_test_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_dependencies(teleop_test_generate_messages_eus _teleop_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_test_geneus)
add_dependencies(teleop_test_geneus teleop_test_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_test_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(teleop_test
  "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_test
)

### Generating Services

### Generating Module File
_generate_module_lisp(teleop_test
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_test
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(teleop_test_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(teleop_test_generate_messages teleop_test_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_dependencies(teleop_test_generate_messages_lisp _teleop_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_test_genlisp)
add_dependencies(teleop_test_genlisp teleop_test_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_test_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(teleop_test
  "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_test
)

### Generating Services

### Generating Module File
_generate_module_nodejs(teleop_test
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_test
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(teleop_test_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(teleop_test_generate_messages teleop_test_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_dependencies(teleop_test_generate_messages_nodejs _teleop_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_test_gennodejs)
add_dependencies(teleop_test_gennodejs teleop_test_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_test_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(teleop_test
  "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_test
)

### Generating Services

### Generating Module File
_generate_module_py(teleop_test
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_test
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(teleop_test_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(teleop_test_generate_messages teleop_test_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/ROS/catkin/src/teleop_test/msg/Num.msg" NAME_WE)
add_dependencies(teleop_test_generate_messages_py _teleop_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_test_genpy)
add_dependencies(teleop_test_genpy teleop_test_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_test_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_test
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(teleop_test_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_test
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(teleop_test_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_test
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(teleop_test_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_test
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(teleop_test_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_test)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_test\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_test
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(teleop_test_generate_messages_py std_msgs_generate_messages_py)
endif()
