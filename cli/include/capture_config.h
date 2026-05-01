#pragma once

#include <cstddef>
#include <string>

namespace open_raw_radar::cli {

enum class TriggerMode {
    kSoftware,
    kHardware,
};

struct NetworkConfig {
    std::string subnet;
    std::string board_suffix;
    std::string host_suffix;
    std::string board_ip;
    std::string host_ip;
};

struct CaptureConfig {
    std::string radar_config_filename;
    std::string dca_config_filename;
    std::string radar_config_path;
    std::string dca_config_path;
    NetworkConfig network;
    std::size_t bytes_per_frame{0};
    std::size_t packets_per_frame{0};
    TriggerMode trigger_mode{TriggerMode::kSoftware};
};

CaptureConfig load_capture_config(const std::string& radar_config_filename,
                                  const std::string& dca_config_filename,
                                  const std::string& subnet_override,
                                  const std::string& host_suffix_override,
                                  const std::string& board_suffix_override);

}  // namespace open_raw_radar::cli
