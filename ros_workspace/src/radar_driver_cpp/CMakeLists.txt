cmake_minimum_required(VERSION 3.22)
project(radar_driver_cpp)

# By default use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_msgs REQUIRED)

# find sources
file(GLOB_RECURSE MMWAVE_SINGLECHIP_SRCS
        "include/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.cpp"
        "include/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.c"
)
file(GLOB_RECURSE MMWAVE_LINK_SRCS
        "include/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink/src/*.c"
)
set(SOURCES
        include/radar_driver_cpp/WzSerialportPlus/Linux/WzSerialportPlus.cpp
        include/radar_driver_cpp/mmwaveDFP_2G/FTDILib/SourceCode/mmwl_port_ftdi.cpp
        include/radar_driver_cpp/pevents/pevents.cpp
        ${MMWAVE_SINGLECHIP_SRCS}
        ${MMWAVE_LINK_SRCS}
)
set(PUBLISHER_CORE_SOURCES
        src/core/publisher_config.cpp
        src/core/publisher_capture.cpp
)
set(PUBLISHER_CONFIG_SOURCES
        src/core/publisher_config.cpp
)

# publisher executable
add_executable(publisher src/nodes/publisher.cpp ${PUBLISHER_CORE_SOURCES} ${SOURCES})
ament_target_dependencies(publisher rclcpp std_msgs my_msgs)
target_link_libraries(publisher ftd2xx)
target_include_directories(publisher PRIVATE
        include
        include/radar_driver_cpp
        include/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example
        include/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink/include
        include/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink
        include/radar_driver_cpp/mmwaveDFP_2G/FTDILib/SourceCode
        include/radar_driver_cpp/mmwaveDFP_2G/firmware
        include/radar_driver_cpp/WzSerialportPlus/Linux
        include/radar_driver_cpp/FTDI_D2XX/Linux
        include/radar_driver_cpp/pevents
        include/radar_driver_cpp/kfifo
        include/radar_driver_cpp/
)

# subscriber executable
add_executable(subscriber src/nodes/subscriber.cpp ${PUBLISHER_CONFIG_SOURCES})
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs my_msgs)
target_link_libraries(subscriber)
target_include_directories(subscriber PRIVATE
        include
        include/radar_driver_cpp/
)

# install launch file
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
        publisher
        subscriber
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
