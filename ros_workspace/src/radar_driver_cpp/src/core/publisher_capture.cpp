#include "radar_driver_cpp/core/publisher_capture.h"

#include <sstream>
#include <stdexcept>
#include <utility>

#include "radar_driver_cpp/transport/ftdi_comm.h"

namespace radar_driver_cpp::core {
namespace {

constexpr uint32_t kPacketsPerFetch = 500;
constexpr int kUdpTimeoutMs = 1500;
constexpr uint32_t kNoDataRetryCount = 5;

void ensure_data_available(const uint32_t first_sequence, uint32_t& no_data_count) {
    if (first_sequence != 0) {
        no_data_count = kNoDataRetryCount;
        return;
    }

    if (no_data_count > 0) {
        --no_data_count;
    }

    if (no_data_count == 0) {
        throw CaptureError(CaptureErrorCode::kNoData, "always no data from DCA, check USB connection");
    }
}

void ensure_chunk_sequence_is_contiguous(const uint32_t first_sequence,
                                         const uint32_t last_sequence,
                                         uint32_t& sequence_shift) {
    const bool first_boundary_ok = (first_sequence - 1 - sequence_shift) % kPacketsPerFetch == 0;
    const bool last_boundary_ok = (last_sequence - sequence_shift) % kPacketsPerFetch == 0;
    if (first_boundary_ok && last_boundary_ok) {
        return;
    }

    sequence_shift = (first_sequence - 1) % kPacketsPerFetch;
    std::ostringstream stream;
    stream << "Packet loss happened: " << first_sequence << " - " << last_sequence;
    throw CaptureError(CaptureErrorCode::kPacketLoss, stream.str());
}

}  // namespace

CaptureError::CaptureError(const CaptureErrorCode code, std::string message)
    : code_(code), message_(std::move(message)) {}

CaptureErrorCode CaptureError::code() const noexcept {
    return code_;
}

const char* CaptureError::what() const noexcept {
    return message_.c_str();
}

RadarCaptureSession::RadarCaptureSession(std::shared_ptr<RadarUDP> radar_udp,
                                         std::string radar_config_path,
                                         std::string dca_config_path,
                                         size_t packets_per_frame)
    : radar_udp_(std::move(radar_udp)),
      radar_config_path_(std::move(radar_config_path)),
      dca_config_path_(std::move(dca_config_path)),
      packets_per_frame_(packets_per_frame) {}

RadarCaptureSession::~RadarCaptureSession() {
    shutdown();
}

void RadarCaptureSession::prepare() {
    AWR2243_sensorStop();
    AWR2243_waitSensorStop();
    radar_udp_->reset_radar();
    radar_udp_->reset_fpga();

    AWR2243_init(radar_config_path_);
    AWR2243_setFrameCfg(0);
    radar_udp_->config_fpga(dca_config_path_);

    radar_udp_->udp_read_thread_init(static_cast<int>(2 * packets_per_frame_));
    radar_udp_->stream_start();
    AWR2243_sensorStart();

    no_data_count_ = kNoDataRetryCount;
    sequence_shift_ = 0;
    state_ = SessionState::kRunning;
}

void RadarCaptureSession::shutdown() {
    if (state_ == SessionState::kStopped) {
        return;
    }

    AWR2243_sensorStop();
    AWR2243_waitSensorStop();
    if (radar_udp_) {
        radar_udp_->stream_stop();
    }
    state_ = SessionState::kStopped;
}

PacketChunk RadarCaptureSession::fetch_next_chunk() {
    PacketChunk chunk;
    chunk.data = RadarUDP::udp_read_thread_get_packets(
        kPacketsPerFetch, kUdpTimeoutMs, true, chunk.first_sequence, chunk.last_sequence);

    ensure_data_available(chunk.first_sequence, no_data_count_);
    if (!chunk.has_data()) {
        return chunk;
    }
    ensure_chunk_sequence_is_contiguous(chunk.first_sequence, chunk.last_sequence, sequence_shift_);
    return chunk;
}

}  // namespace radar_driver_cpp::core
