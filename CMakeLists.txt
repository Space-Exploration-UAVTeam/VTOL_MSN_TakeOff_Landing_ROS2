cmake_minimum_required(VERSION 3.8)
project(vtol_msn_takeoff_landing)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
########### 
## Build ##
###########
add_executable(vtol_msn_takeoff_landing src/vtol_msn_takeoff_landing.cpp)
ament_target_dependencies(vtol_msn_takeoff_landing PUBLIC
 geometry_msgs
 rclcpp
 sensor_msgs
 std_msgs
 nav_msgs
 apriltag_ros2_interfaces
 )

#############
## Install ##
#############

install(TARGETS vtol_msn_takeoff_landing
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME})
  
ament_package()