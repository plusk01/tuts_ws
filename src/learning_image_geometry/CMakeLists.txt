cmake_minimum_required(VERSION 2.8.3)
project(learning_image_geometry)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  image_view
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories( include ${catkin_INCLUDE_DIRS} )

# frame drawer node
add_executable(draw_frames src/main.cpp src/draw_frames.cpp)
target_link_libraries(draw_frames ${catkin_LIBRARIES})