# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lenna_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilenna_msgs:/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lenna_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_custom_target(_lenna_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lenna_msgs" "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" "std_msgs/Header:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:nav_msgs/Odometry:geometry_msgs/Twist:sensor_msgs/Imu:geometry_msgs/Point:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lenna_msgs
  "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lenna_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(lenna_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lenna_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lenna_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lenna_msgs_generate_messages lenna_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_dependencies(lenna_msgs_generate_messages_cpp _lenna_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lenna_msgs_gencpp)
add_dependencies(lenna_msgs_gencpp lenna_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lenna_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lenna_msgs
  "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lenna_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(lenna_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lenna_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lenna_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lenna_msgs_generate_messages lenna_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_dependencies(lenna_msgs_generate_messages_eus _lenna_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lenna_msgs_geneus)
add_dependencies(lenna_msgs_geneus lenna_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lenna_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lenna_msgs
  "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lenna_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(lenna_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lenna_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lenna_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lenna_msgs_generate_messages lenna_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_dependencies(lenna_msgs_generate_messages_lisp _lenna_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lenna_msgs_genlisp)
add_dependencies(lenna_msgs_genlisp lenna_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lenna_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lenna_msgs
  "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lenna_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lenna_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lenna_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lenna_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lenna_msgs_generate_messages lenna_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_dependencies(lenna_msgs_generate_messages_nodejs _lenna_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lenna_msgs_gennodejs)
add_dependencies(lenna_msgs_gennodejs lenna_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lenna_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lenna_msgs
  "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lenna_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(lenna_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lenna_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lenna_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lenna_msgs_generate_messages lenna_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lenna/repositories/Lenna-Mobile-Robot-ONE/lenna_ws/src/lenna_msgs/msg/OdomInformation.msg" NAME_WE)
add_dependencies(lenna_msgs_generate_messages_py _lenna_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lenna_msgs_genpy)
add_dependencies(lenna_msgs_genpy lenna_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lenna_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lenna_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lenna_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(lenna_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(lenna_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lenna_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(lenna_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lenna_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lenna_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(lenna_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(lenna_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lenna_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(lenna_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lenna_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lenna_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(lenna_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(lenna_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lenna_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(lenna_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lenna_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lenna_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(lenna_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(lenna_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lenna_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(lenna_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lenna_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lenna_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lenna_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(lenna_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(lenna_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lenna_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(lenna_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
