# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reactive_robot: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ireactive_robot:/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reactive_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_custom_target(_reactive_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reactive_robot" "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" ""
)

get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_custom_target(_reactive_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reactive_robot" "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reactive_robot
)
_generate_msg_cpp(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reactive_robot
)

### Generating Services

### Generating Module File
_generate_module_cpp(reactive_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reactive_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reactive_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reactive_robot_generate_messages reactive_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_cpp _reactive_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_cpp _reactive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reactive_robot_gencpp)
add_dependencies(reactive_robot_gencpp reactive_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reactive_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reactive_robot
)
_generate_msg_eus(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reactive_robot
)

### Generating Services

### Generating Module File
_generate_module_eus(reactive_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reactive_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(reactive_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(reactive_robot_generate_messages reactive_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_eus _reactive_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_eus _reactive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reactive_robot_geneus)
add_dependencies(reactive_robot_geneus reactive_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reactive_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reactive_robot
)
_generate_msg_lisp(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reactive_robot
)

### Generating Services

### Generating Module File
_generate_module_lisp(reactive_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reactive_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(reactive_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(reactive_robot_generate_messages reactive_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_lisp _reactive_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_lisp _reactive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reactive_robot_genlisp)
add_dependencies(reactive_robot_genlisp reactive_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reactive_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reactive_robot
)
_generate_msg_nodejs(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reactive_robot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(reactive_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reactive_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(reactive_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(reactive_robot_generate_messages reactive_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_nodejs _reactive_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_nodejs _reactive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reactive_robot_gennodejs)
add_dependencies(reactive_robot_gennodejs reactive_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reactive_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot
)
_generate_msg_py(reactive_robot
  "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot
)

### Generating Services

### Generating Module File
_generate_module_py(reactive_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reactive_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reactive_robot_generate_messages reactive_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/collision.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_py _reactive_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jkleiber/intro_robotics_ws/src/reactive_robot/msg/obstacle.msg" NAME_WE)
add_dependencies(reactive_robot_generate_messages_py _reactive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reactive_robot_genpy)
add_dependencies(reactive_robot_genpy reactive_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reactive_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reactive_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reactive_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reactive_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(reactive_robot_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reactive_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reactive_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(reactive_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(reactive_robot_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reactive_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reactive_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(reactive_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(reactive_robot_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reactive_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reactive_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(reactive_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(reactive_robot_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reactive_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reactive_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(reactive_robot_generate_messages_py geometry_msgs_generate_messages_py)
endif()
