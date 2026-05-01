cmake_minimum_required(VERSION 3.22)
project(my_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TimeStamp.msg"
    "msg/State.msg"
    "msg/RawRadar.msg"
    DEPENDENCIES geometry_msgs)

ament_package()
