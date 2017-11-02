## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Ofast -fwhole-program)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

cmake_minimum_required(VERSION 2.8.12)
project(warper)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")



# Find rquired packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  actionlib
  image_transport
  sensor_msgs

  )

catkin_package(
 #INCLUDE_DIRS include
 #  LIBRARIES warper
 #  CATKIN_DEPENDS sensor_msgs
 #  DEPENDS system_lib
 )


include_directories(include
 ${catkin_INCLUDE_DIRS}
 )

# Enable OPENCV
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

add_executable(warper src/warper_node.cpp)


target_link_libraries(warper
  ${catkin_LIBRARIES}
  )


  #install(
  #  DIRECTORY include/${PROJECT_NAME}/
  # DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #  FILES_MATCHING PATTERN "*.h"
  #)


