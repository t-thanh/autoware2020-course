cmake_minimum_required(VERSION 3.5)
project(lane_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(lane_msgs
  "msg/LaneMarking.msg"
  "msg/LaneMarkingArray.msg"
  "msg/LaneMarkingArrayBoth.msg"
  "msg/LaneMarkingProjected.msg"
  "msg/LaneMarkingProjectedArray.msg"
  "msg/LaneMarkingProjectedArrayBoth.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
