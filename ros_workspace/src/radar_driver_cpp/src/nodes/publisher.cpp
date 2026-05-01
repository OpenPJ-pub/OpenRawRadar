#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "my_msgs/msg/raw_radar.hpp"
#include "radar_driver_cpp/core/publisher_capture.h"
#include "radar_driver_cpp/core/publisher_config.h"

namespace {

constexpr uint32_t kPacketsPerFetch = 500;
constexpr const char* kDefaultRadarConfigFilename = "AWR2243_mmwaveconfig_max15.txt";
constexpr const char* kDefaultDcaConfigFilename = "dca_config.txt";
constexpr const char* kRadarTopic = "/radar/sensor_data";

std::string format_local_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    std::ostringstream stream;
    stream << (ns / 1000000000LL)
           << "."
           << std::setw(9)
           << std::setfill('0')
           << (ns % 1000000000LL);
    return stream.str();
}

}  // namespace

class RadarPublisherNode final : public rclcpp::Node {
public:
    RadarPublisherNode()
        : Node("RadarPublisher"),
          capture_running_(false),
          frame_count_(0) {
        initialize_parameters();
        initialize_capture_configuration();
        initialize_publisher();
        capture_thread_ = std::thread(&RadarPublisherNode::run_capture_loop, this);
    }

    ~RadarPublisherNode() override {
        stop_capture();
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Radar publisher closed.");
    }

private:
    void initialize_parameters() {
        declare_parameter<std::string>("radar_config_filename", kDefaultRadarConfigFilename);
        declare_parameter<std::string>("dca_config_filename", kDefaultDcaConfigFilename);
        get_parameter("radar_config_filename", radar_config_filename_);
        get_parameter("dca_config_filename", dca_config_filename_);
        RCLCPP_INFO(get_logger(), "Radar config: %s", radar_config_filename_.c_str());
        RCLCPP_INFO(get_logger(), "DCA config: %s", dca_config_filename_.c_str());
    }

    void initialize_capture_configuration() {
        publisher_config_ = radar_driver_cpp::core::load_publisher_config(
            radar_config_filename_, dca_config_filename_);

        auto radar_udp = std::make_shared<RadarUDP>(publisher_config_.network.board_ip,
                                                    publisher_config_.network.host_ip,
                                                    4096,
                                                    4098);
        capture_session_ = std::make_unique<radar_driver_cpp::core::RadarCaptureSession>(
            std::move(radar_udp),
            publisher_config_.radar_config_path,
            publisher_config_.dca_config_path,
            publisher_config_.packets_per_frame);
    }

    void initialize_publisher() {
        raw_radar_publisher_ = create_publisher<my_msgs::msg::RawRadar>(
            kRadarTopic, radar_driver_cpp::core::build_publisher_qos());
    }

    void start_capture() {
        capture_session_->prepare();
        capture_running_ = true;
        if (publisher_config_.trigger_mode == radar_driver_cpp::core::TriggerMode::kHardware) {
            RCLCPP_WARN(get_logger(), "hardware trigger timestamp hook is reserved but not implemented");
        }
        RCLCPP_INFO(get_logger(), "radar is started");
    }

    void stop_capture() {
        capture_running_ = false;
        if (capture_session_) {
            capture_session_->shutdown();
        }
    }

    void publish_chunk(const radar_driver_cpp::core::PacketChunk& chunk) {
        my_msgs::msg::RawRadar radar_msg;
        radar_msg.data = chunk.data;
        raw_radar_publisher_->publish(radar_msg);
    }

    void log_completed_frame_if_needed(uint32_t& packet_count) {
        if (packet_count < publisher_config_.packets_per_frame) {
            return;
        }

        packet_count -= static_cast<uint32_t>(publisher_config_.packets_per_frame);
        ++frame_count_;
        RCLCPP_INFO(get_logger(),
                    "[config=%s] frame=%u ts=%s",
                    publisher_config_.radar_config_filename.c_str(),
                    frame_count_,
                    format_local_timestamp().c_str());
    }

    void run_capture_loop() {
        try {
            start_capture();

            uint32_t packet_count = 0;
            while (capture_running_) {
                const auto chunk = capture_session_->fetch_next_chunk();
                if (!chunk.has_data()) {
                    RCLCPP_WARN(get_logger(), "no udp data from DCA");
                    continue;
                }
                packet_count += kPacketsPerFetch;
                log_completed_frame_if_needed(packet_count);
                publish_chunk(chunk);
            }
        } catch (const radar_driver_cpp::core::CaptureError& error) {
            RCLCPP_ERROR(get_logger(), "Radar capture thread error: %s", error.what());
            stop_capture();
            rclcpp::shutdown();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Radar capture thread error: %s", error.what());
            stop_capture();
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<my_msgs::msg::RawRadar>::SharedPtr raw_radar_publisher_;
    radar_driver_cpp::core::PublisherConfig publisher_config_{};
    std::unique_ptr<radar_driver_cpp::core::RadarCaptureSession> capture_session_;

    std::thread capture_thread_;
    std::atomic<bool> capture_running_;
    uint32_t frame_count_;
    std::string radar_config_filename_;
    std::string dca_config_filename_;
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<RadarPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
