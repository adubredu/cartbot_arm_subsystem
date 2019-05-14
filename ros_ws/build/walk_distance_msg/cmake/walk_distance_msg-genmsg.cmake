# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "walk_distance_msg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iwalk_distance_msg:/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(walk_distance_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_custom_target(_walk_distance_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "walk_distance_msg" "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(walk_distance_msg
  "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/walk_distance_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(walk_distance_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/walk_distance_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(walk_distance_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(walk_distance_msg_generate_messages walk_distance_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_dependencies(walk_distance_msg_generate_messages_cpp _walk_distance_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(walk_distance_msg_gencpp)
add_dependencies(walk_distance_msg_gencpp walk_distance_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS walk_distance_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(walk_distance_msg
  "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/walk_distance_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(walk_distance_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/walk_distance_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(walk_distance_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(walk_distance_msg_generate_messages walk_distance_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_dependencies(walk_distance_msg_generate_messages_eus _walk_distance_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(walk_distance_msg_geneus)
add_dependencies(walk_distance_msg_geneus walk_distance_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS walk_distance_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(walk_distance_msg
  "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/walk_distance_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(walk_distance_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/walk_distance_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(walk_distance_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(walk_distance_msg_generate_messages walk_distance_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_dependencies(walk_distance_msg_generate_messages_lisp _walk_distance_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(walk_distance_msg_genlisp)
add_dependencies(walk_distance_msg_genlisp walk_distance_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS walk_distance_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(walk_distance_msg
  "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/walk_distance_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(walk_distance_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/walk_distance_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(walk_distance_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(walk_distance_msg_generate_messages walk_distance_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_dependencies(walk_distance_msg_generate_messages_nodejs _walk_distance_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(walk_distance_msg_gennodejs)
add_dependencies(walk_distance_msg_gennodejs walk_distance_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS walk_distance_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(walk_distance_msg
  "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/walk_distance_msg
)

### Generating Services

### Generating Module File
_generate_module_py(walk_distance_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/walk_distance_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(walk_distance_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(walk_distance_msg_generate_messages walk_distance_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/bill_ros/rascapp_robot/ros_ws/src/walk_distance_msg/msg/Walk.msg" NAME_WE)
add_dependencies(walk_distance_msg_generate_messages_py _walk_distance_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(walk_distance_msg_genpy)
add_dependencies(walk_distance_msg_genpy walk_distance_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS walk_distance_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/walk_distance_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/walk_distance_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(walk_distance_msg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(walk_distance_msg_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(walk_distance_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/walk_distance_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/walk_distance_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(walk_distance_msg_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(walk_distance_msg_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(walk_distance_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/walk_distance_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/walk_distance_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(walk_distance_msg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(walk_distance_msg_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(walk_distance_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/walk_distance_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/walk_distance_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(walk_distance_msg_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(walk_distance_msg_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(walk_distance_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/walk_distance_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/walk_distance_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/walk_distance_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(walk_distance_msg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(walk_distance_msg_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(walk_distance_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
