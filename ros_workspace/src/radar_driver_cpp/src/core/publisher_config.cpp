#include "radar_driver_cpp/core/publisher_config.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "radar_driver_cpp/config/awr2243_config_parser.h"

namespace radar_driver_cpp::core {
namespace {

constexpr size_t kPacketPayloadBytes = 1456;
constexpr const char* kDefaultSubnet = "33";
constexpr const char* kDefaultHostSuffix = "30";
constexpr const char* kDefaultBoardSuffix = "180";
namespace fs = std::filesystem;

struct ParsedConfigEntry {
    std::string key;
    std::string value;
};

std::string resolve_config_path(const std::string& radar_config_filename) {
    if (radar_config_filename.empty()) {
        throw std::runtime_error("Empty config path");
    }
    if (fs::exists(radar_config_filename)) {
        return fs::absolute(radar_config_filename).string();
    }

    const fs::path candidate_paths[] = {
        fs::path("configs") / radar_config_filename,
        fs::path("../configs") / radar_config_filename,
        fs::path("/configs") / radar_config_filename,
    };

    for (const auto& candidate : candidate_paths) {
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }

    throw std::runtime_error("Unable to resolve config path: " + radar_config_filename);
}

std::string resolve_dca_config_path(const std::string& dca_config_filename) {
    return resolve_config_path(dca_config_filename);
}

std::string resolve_subnet() {
    if (const char* env_subnet = std::getenv("SUBNET")) {
        return env_subnet;
    }
    return kDefaultSubnet;
}

std::string resolve_address_suffix(const char* env_name, const char* default_suffix) {
    if (const char* env_value = std::getenv(env_name)) {
        return env_value;
    }
    return default_suffix;
}

RadarNetworkConfig build_network_config() {
    RadarNetworkConfig network;
    network.subnet = resolve_subnet();
    network.host_suffix = resolve_address_suffix("RADAR_HOST_SUFFIX", kDefaultHostSuffix);
    network.board_suffix = resolve_address_suffix("RADAR_BOARD_SUFFIX", kDefaultBoardSuffix);
    network.board_ip = "192.168." + network.subnet + "." + network.board_suffix;
    network.host_ip = "192.168." + network.subnet + "." + network.host_suffix;
    return network;
}

size_t compute_bytes_per_frame(const ADC_PARAMS& adc_params) {
    return static_cast<size_t>(adc_params.chirps) *
           adc_params.rx *
           adc_params.tx *
           adc_params.IQ *
           adc_params.samples *
           adc_params.bytes;
}

size_t compute_packets_per_frame(const size_t bytes_per_frame) {
    return (bytes_per_frame + kPacketPayloadBytes - 1) / kPacketPayloadBytes;
}

TriggerMode resolve_trigger_mode(const ADC_PARAMS& adc_params) {
    return adc_params.triggerSelect == "hardware" ? TriggerMode::kHardware : TriggerMode::kSoftware;
}

void trim(std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        value.clear();
        return;
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    value = value.substr(first, last - first + 1);
}

bool parse_config_entry(const std::string& line, ParsedConfigEntry& entry) {
    const auto delimiter = line.find('=');
    if (delimiter == std::string::npos) {
        return false;
    }

    entry.key = line.substr(0, delimiter);
    entry.value = line.substr(delimiter + 1);
    const auto semicolon = entry.value.find(';');
    if (semicolon != std::string::npos) {
        entry.value = entry.value.substr(0, semicolon);
    }
    trim(entry.key);
    trim(entry.value);
    return !entry.key.empty() && !entry.value.empty();
}

void validate_num_adc_samples(const std::string& radar_config_path, const ADC_PARAMS& adc_params) {
    std::ifstream config(radar_config_path);
    if (!config.is_open()) {
        throw std::runtime_error("Unable to open config file");
    }

    int profile_id_counter = 0;
    int profile_num_adc_samples = -1;
    int frame_num_adc_samples = -1;
    std::string line;
    ParsedConfigEntry entry;

    while (std::getline(config, line)) {
        trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        if (!parse_config_entry(line, entry)) {
            continue;
        }

        if (entry.key == "rxGain") {
            ++profile_id_counter;
            continue;
        }

        if (entry.key != "numAdcSamples") {
            continue;
        }

        const int parsed_value = std::stoi(entry.value);
        if (profile_id_counter == 0) {
            profile_num_adc_samples = parsed_value;
        } else {
            frame_num_adc_samples = parsed_value;
        }
    }

    if (profile_num_adc_samples < 0) {
        throw std::runtime_error("Missing profile numAdcSamples in radar config");
    }

    if (profile_num_adc_samples != adc_params.samples) {
        throw std::runtime_error("Profile numAdcSamples does not match parsed ADC sample count");
    }

    if (frame_num_adc_samples < 0) {
        return;
    }

    const int expected_frame_num_adc_samples =
        adc_params.IQ == 2 ? profile_num_adc_samples * 2 : profile_num_adc_samples;
    if (frame_num_adc_samples != expected_frame_num_adc_samples) {
        std::ostringstream stream;
        stream << "Frame numAdcSamples mismatch: expected "
               << expected_frame_num_adc_samples
               << ", got "
               << frame_num_adc_samples;
        throw std::runtime_error(stream.str());
    }
}

}  // namespace

rclcpp::QoS build_publisher_qos() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(600));
    qos.reliable();
    qos.durability_volatile();
    qos.lifespan(rclcpp::Duration(1, 0));
    return qos;
}

PublisherConfig load_publisher_config(const std::string& radar_config_filename,
                                      const std::string& dca_config_filename) {
    PublisherConfig config;
    config.radar_config_filename = radar_config_filename;
    config.dca_config_filename = dca_config_filename;
    config.radar_config_path = resolve_config_path(radar_config_filename);
    config.dca_config_path = resolve_dca_config_path(dca_config_filename);
    const ADC_PARAMS adc_params = awr2243_read_config(config.radar_config_path);
    validate_num_adc_samples(config.radar_config_path, adc_params);
    config.network = build_network_config();
    config.bytes_per_frame = compute_bytes_per_frame(adc_params);
    config.packets_per_frame = compute_packets_per_frame(config.bytes_per_frame);
    config.trigger_mode = resolve_trigger_mode(adc_params);
    return config;
}

}  // namespace radar_driver_cpp::core
