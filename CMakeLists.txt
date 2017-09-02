## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Ofast -fwhole-program)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.12)
project(warper)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Define path of darknet folder here.
#find_path(DARKNET_PATH
# NAMES "README.md"
#  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../darknet/")
#message("Darknet path dir = ${DARKNET_PATH}")
#add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Find CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_20,code=[sm_20,sm_21]
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
MESSAGE ( STATUS "Searching for X11..." )
FIND_PACKAGE ( X11 REQUIRED )
IF ( X11_FOUND )
  INCLUDE_DIRECTORIES ( ${X11_INCLUDE_DIR} )
  LINK_LIBRARIES ( ${X11_LIBRARIES} )
  MESSAGE ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  MESSAGE ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
ENDIF ( X11_FOUND )

# Find rquired packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    darknet_ros_msgs
    image_transport
    sensor_msgs

)

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    #cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    image_transport
  DEPENDS
    Boost
)

if (CUDA_FOUND)
  include_directories(
#    ${DARKNET_PATH}/src
#    ${DARKNET_PATH}/include
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )

  link_directories(
    /usr/local/cuda/lib64
  )

  cuda_add_library(${PROJECT_NAME}_lib
    #src/YoloObjectDetector.cpp
    #src/object_detector_demo.cpp
    #src/image_interface.c

    #${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/im2col.c

    #${DARKNET_PATH}/src/crop_layer_kernels.cu
  )

  target_link_libraries(${PROJECT_NAME}_lib
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  cuda_add_executable(${PROJECT_NAME}
    src/warper_node.cpp
    src/GPU_Warper.cu
  )

  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib
  )

  install(
    TARGETS ${PROJECT_NAME}_lib
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )
endif()


