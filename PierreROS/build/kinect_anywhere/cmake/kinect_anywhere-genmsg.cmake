# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kinect_anywhere: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ikinect_anywhere:/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kinect_anywhere_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg" NAME_WE)
add_custom_target(_kinect_anywhere_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kinect_anywhere" "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg" "geometry_msgs/Point:std_msgs/Header:kinect_anywhere/JointPositionAndState"
)

get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg" NAME_WE)
add_custom_target(_kinect_anywhere_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kinect_anywhere" "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg" "geometry_msgs/Point:kinect_anywhere/Body:std_msgs/Header:kinect_anywhere/JointPositionAndState"
)

get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg" NAME_WE)
add_custom_target(_kinect_anywhere_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kinect_anywhere" "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_cpp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_cpp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere
)

### Generating Services

### Generating Module File
_generate_module_cpp(kinect_anywhere
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kinect_anywhere_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kinect_anywhere_generate_messages kinect_anywhere_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_cpp _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_cpp _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_cpp _kinect_anywhere_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect_anywhere_gencpp)
add_dependencies(kinect_anywhere_gencpp kinect_anywhere_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect_anywhere_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_lisp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_lisp(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere
)

### Generating Services

### Generating Module File
_generate_module_lisp(kinect_anywhere
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kinect_anywhere_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kinect_anywhere_generate_messages kinect_anywhere_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_lisp _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_lisp _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_lisp _kinect_anywhere_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect_anywhere_genlisp)
add_dependencies(kinect_anywhere_genlisp kinect_anywhere_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect_anywhere_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_py(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere
)
_generate_msg_py(kinect_anywhere
  "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere
)

### Generating Services

### Generating Module File
_generate_module_py(kinect_anywhere
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kinect_anywhere_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kinect_anywhere_generate_messages kinect_anywhere_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/Body.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_py _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/BodyFrame.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_py _kinect_anywhere_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg/JointPositionAndState.msg" NAME_WE)
add_dependencies(kinect_anywhere_generate_messages_py _kinect_anywhere_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect_anywhere_genpy)
add_dependencies(kinect_anywhere_genpy kinect_anywhere_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect_anywhere_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect_anywhere
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(kinect_anywhere_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(kinect_anywhere_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect_anywhere
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(kinect_anywhere_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(kinect_anywhere_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect_anywhere
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(kinect_anywhere_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(kinect_anywhere_generate_messages_py std_msgs_generate_messages_py)
endif()
