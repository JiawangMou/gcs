# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mav_comm_driver: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imav_comm_driver:/home/moujiawang/gcs/src/mav_comm_driver/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mav_comm_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_custom_target(_mav_comm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mav_comm_driver" "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" "geometry_msgs/Twist:std_msgs/Header:geometry_msgs/TwistWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:nav_msgs/Odometry:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_custom_target(_mav_comm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mav_comm_driver" "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_comm_driver
)
_generate_msg_cpp(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_comm_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(mav_comm_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_comm_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mav_comm_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mav_comm_driver_generate_messages mav_comm_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_cpp _mav_comm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_cpp _mav_comm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_comm_driver_gencpp)
add_dependencies(mav_comm_driver_gencpp mav_comm_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_comm_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_comm_driver
)
_generate_msg_eus(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_comm_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(mav_comm_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_comm_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mav_comm_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mav_comm_driver_generate_messages mav_comm_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_eus _mav_comm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_eus _mav_comm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_comm_driver_geneus)
add_dependencies(mav_comm_driver_geneus mav_comm_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_comm_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_comm_driver
)
_generate_msg_lisp(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_comm_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(mav_comm_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_comm_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mav_comm_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mav_comm_driver_generate_messages mav_comm_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_lisp _mav_comm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_lisp _mav_comm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_comm_driver_genlisp)
add_dependencies(mav_comm_driver_genlisp mav_comm_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_comm_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_comm_driver
)
_generate_msg_nodejs(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_comm_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mav_comm_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_comm_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mav_comm_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mav_comm_driver_generate_messages mav_comm_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_nodejs _mav_comm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_nodejs _mav_comm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_comm_driver_gennodejs)
add_dependencies(mav_comm_driver_gennodejs mav_comm_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_comm_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver
)
_generate_msg_py(mav_comm_driver
  "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver
)

### Generating Services

### Generating Module File
_generate_module_py(mav_comm_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mav_comm_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mav_comm_driver_generate_messages mav_comm_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/MAVStatus.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_py _mav_comm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moujiawang/gcs/src/mav_comm_driver/msg/ModeConfig.msg" NAME_WE)
add_dependencies(mav_comm_driver_generate_messages_py _mav_comm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_comm_driver_genpy)
add_dependencies(mav_comm_driver_genpy mav_comm_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_comm_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_comm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_comm_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(mav_comm_driver_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mav_comm_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(mav_comm_driver_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_comm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_comm_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(mav_comm_driver_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mav_comm_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(mav_comm_driver_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_comm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_comm_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(mav_comm_driver_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mav_comm_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(mav_comm_driver_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_comm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_comm_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(mav_comm_driver_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mav_comm_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(mav_comm_driver_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_comm_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(mav_comm_driver_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mav_comm_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(mav_comm_driver_generate_messages_py nav_msgs_generate_messages_py)
endif()
