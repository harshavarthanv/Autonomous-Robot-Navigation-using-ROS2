cmake_minimum_required(VERSION 3.8)
project(group16_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder whish has all the header files
include_directories(include/group16_final
  ${rosgraph_msgs_INCLUDE_DIRS}
  )

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  ros2_aruco_interfaces
  mage_msgs
  nav_msgs
  rosgraph_msgs
  nav2_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

#add execulable
add_executable(listen src/listen.cpp)
ament_target_dependencies(listen ${FRAME_DEMO_INCLUDE_DEPENDS})

#initialze execulables
install(TARGETS
listen
DESTINATION lib/${PROJECT_NAME}
)

#include required path folders
install(DIRECTORY include 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
