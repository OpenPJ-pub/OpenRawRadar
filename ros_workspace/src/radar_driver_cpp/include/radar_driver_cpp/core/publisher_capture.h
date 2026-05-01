#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "radar_driver_cpp/transport/radar_udp.h"

namespace radar_driver_cpp::core {

struct PacketChunk {
    std::vector<uint8_t> data;
    uint32_t first_sequence{0};
    uint32_t last_sequence{0};

    bool has_data() const noexcept {
        return first_sequence != 0;
    }
};

enum class CaptureErrorCode {
    kNoData,
    kPacketLoss,
};

class CaptureError final : public std::exception {
public:
    CaptureError(CaptureErrorCode code, std::string message);

    CaptureErrorCode code() const noexcept;
    const char* what() const noexcept override;

private:
    CaptureErrorCode code_;
    std::string message_;
};

class RadarCaptureSession {
public:
    RadarCaptureSession(std::shared_ptr<RadarUDP> radar_udp,
                        std::string radar_config_path,
                        std::string dca_config_path,
                        size_t packets_per_frame);
    ~RadarCaptureSession();

    void prepare();
    void shutdown();
    PacketChunk fetch_next_chunk();

private:
    enum class SessionState {
        kStopped,
        kRunning,
    };

    std::shared_ptr<RadarUDP> radar_udp_;
    std::string radar_config_path_;
    std::string dca_config_path_;
    size_t packets_per_frame_{0};
    uint32_t no_data_count_{5};
    uint32_t sequence_shift_{0};
    SessionState state_{SessionState::kStopped};
};

}  // namespace radar_driver_cpp::core
