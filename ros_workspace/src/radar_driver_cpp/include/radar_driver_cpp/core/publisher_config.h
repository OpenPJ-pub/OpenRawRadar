#pragma once

#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace radar_driver_cpp::core {

enum class TriggerMode {
    kSoftware,
    kHardware,
};

struct RadarNetworkConfig {
    std::string subnet;
    std::string board_suffix;
    std::string host_suffix;
    std::string board_ip;
    std::string host_ip;
};

struct PublisherConfig {
    std::string radar_config_filename;
    std::string dca_config_filename;
    std::string radar_config_path;
    std::string dca_config_path;
    RadarNetworkConfig network;
    size_t bytes_per_frame{0};
    size_t packets_per_frame{0};
    TriggerMode trigger_mode{TriggerMode::kSoftware};
};

rclcpp::QoS build_publisher_qos();
PublisherConfig load_publisher_config(const std::string& radar_config_filename,
                                      const std::string& dca_config_filename);

}  // namespace radar_driver_cpp::core
