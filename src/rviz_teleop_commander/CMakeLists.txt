## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)

project(rviz_teleop_commander)

add_compile_options(-std=c++11)

if(BODY_FRAME STREQUAL 2WING)
  message(INFO "
    Body frame: two wing.
  ")
  add_definitions(-DTWO_WING)
else()
  if(BODY_FRAME STREQUAL 4WING)
    add_definitions(-DFOUR_WING)
    message(INFO "
    Body frame: four wing.
    ")
  else()
    message(FATAL_ERROR "
    Please specify the body frame using
      -DBODY_FRAME=2WING or 4WING
    ")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS 
 std_msgs
 geometry_msgs
 message_generation
 rviz
 tf
 )

add_message_files(FILES
  PID.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  geometry_msgs
  tf
  )


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  )
  
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(
  MOC_FILES
  include/rviz_teleop_commander/teleop_pad.h
  include/rviz_teleop_commander/joystick_widget.h
)

## add_dependencies(vicon_mav_data_save ${PROJECT_NAME}_generate_messages_cpp)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/teleop_pad.cpp
  src/joystick_widget.cpp
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

add_executable(vicon_mav_data_save src/vicon_mav_data_save.cpp)
target_link_libraries(vicon_mav_data_save
  ${catkin_LIBRARIES}
)
add_dependencies(
  vicon_mav_data_save
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_executable(vicon_transfer src/vicon_transfer.cpp)
target_link_libraries(vicon_transfer
  ${catkin_LIBRARIES}
)
add_dependencies(
  vicon_transfer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#install(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



