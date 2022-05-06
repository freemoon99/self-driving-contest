cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])

project(m_race_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv;/usr/include")

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv_line_detection
  CATKIN_DEPENDS rospy roscpp std_msgs
 # DEPENDS system_lib
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})


add_executable(race_opencv_vision_node src/race_vision.cpp)
add_dependencies(race_opencv_vision_node ${race_opencv_vision_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(race_opencv_vision_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


