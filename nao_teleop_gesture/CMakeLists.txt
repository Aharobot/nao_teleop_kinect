cmake_minimum_required(VERSION 2.8.3)
project(nao_teleop_gesture)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set( ROS_CXX_DEPENDENCIES
    roscpp
    std_msgs
    naoqi_msgs
    sensor_msgs
    actionlib
    tf
    geometry_msgs
    skeleton_tracker
    )

# Find catkin and all ROS required packages
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} ${ROS_CXX_DEPENDENCIES} cv_bridge image_transport)

#Added by Igor Rodriguez
find_package(orocos_kdl REQUIRED)
# Find the absolute path to the orocos-kdl library
find_library(orocos_kdl_LIBRARY NAMES ${orocos_kdl_LIBRARIES} PATHS ${orocos_kdl_LIBRARY_DIRS} NO_DEFAULT_PATH)
set(orocos_kdl_LIBRARIES ${orocos_kdl_LIBRARY})
## End 


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} /usr/local/lib ${orocos_kdl_LIBRARIES})

catkin_package(
  #INCLUDE_DIRS include /usr/local/include
  LIBRARIES nao_teleop_gesture
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
  DEPENDS system_lib orocos_kdl cv_bridge image_transport opencv2
)

###########
## Build ##
###########


## Declare a cpp library
# add_library(nao_teleop_gesture
#   src/${PROJECT_NAME}/nao_teleop_gesture.cpp
# )

## Declare a cpp executable
add_executable(nao_motion_control src/nao_motion_control.cpp)
add_executable(nao_arm_control src/nao_arm_control.cpp)
add_executable(nao_head_control src/nao_head_control.cpp)
add_executable(nao_hand_control src/nao_hand_control.cpp)
#add_executable(froga src/froga.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(nao_teleop_gesture_node nao_teleop_gesture_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nao_motion_control ${catkin_LIBRARIES})
target_link_libraries(nao_arm_control ${catkin_LIBRARIES} orocos-kdl)
target_link_libraries(nao_head_control ${catkin_LIBRARIES})
target_link_libraries(nao_hand_control ${catkin_LIBRARIES} orocos-kdl)

add_dependencies(nao_motion_control  ${catkin_EXPORTED_TARGETS})
add_dependencies(nao_arm_control  ${catkin_EXPORTED_TARGETS})
add_dependencies(nao_head_control  ${catkin_EXPORTED_TARGETS})
add_dependencies(nao_hand_control  ${catkin_EXPORTED_TARGETS})
#add_dependencies(froga  ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS nao_teleop_kinect nao_teleop_kinect_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nao_teleop_kinect.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
