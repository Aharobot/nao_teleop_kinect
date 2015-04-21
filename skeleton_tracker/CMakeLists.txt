
cmake_minimum_required(VERSION 2.8.3)
project(skeleton_tracker)


set( ROS_CXX_DEPENDENCIES
   geometry_msgs
   roscpp
   roslib
   tf
   message_generation
   std_msgs
   sensor_msgs
	 
)

# Find catkin and all ROS required packages
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} cv_bridge image_transport orocos_kdl)
FIND_PACKAGE( OpenCV REQUIRED )

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

# Find Nite
find_path(Nite_INCLUDEDIR
	  NAMES XnVNite.h
	  HINTS /usr/include/nite /usr/local/include/nite)
find_library(Nite_LIBRARY
	     NAMES XnVNite_1_3_1
	     HINTS /usr/lib /usr/local/lib
	     PATH_SUFFIXES lib) 

# Added by Igor
add_message_files(
  FILES
  Skeleton.msg
)
add_service_files(
  FILES
  SkeletonState.srv
)

generate_messages()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(${catkin_INCLUDEDIR}
				${Boost_INCLUDE_DIRS}
		    ${OpenNI_INCLUDEDIR}
		    ${Nite_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES} message_runtime
  DEPENDS cv_bridge image_transport opencv2
)

## Declare a cpp executable
add_executable(skeleton_tracker src/skeleton_tracker.cpp)
add_executable(openni_skeleton src/openni_skeleton.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(skeleton_tracker ${catkin_LIBRARIES}
				 ${OpenNI_LIBRARIES}
				 ${Nite_LIBRARY}
				 ${orocos_kdl_LIBRARIES})
target_link_libraries(openni_skeleton ${catkin_LIBRARIES})


add_dependencies(skeleton_tracker geometry_msgs_gencpp)
add_dependencies(openni_skeleton  ${catkin_EXPORTED_TARGETS})


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
    
install(TARGETS skeleton_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES skeleton_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})











