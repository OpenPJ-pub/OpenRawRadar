#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "my_msgs/msg/raw_radar.hpp"
#include "radar_driver_cpp/config/awr2243_config_parser.h"
#include "radar_driver_cpp/core/publisher_config.h"

namespace {

constexpr const char* kRadarInputTopic = "/radar/sensor_data";
constexpr const char* kRdMapTopic = "/radar/rd_map";
constexpr const char* kPointCloudTopic = "/radar/points";
constexpr const char* kDefaultRadarConfigFilename = "AWR2243_mmwaveconfig_max15.txt";
constexpr uint32_t kPacketSize = 1466;
constexpr uint32_t kPacketHeaderBytes = 10;
constexpr uint32_t kPacketPayloadBytes = 1456;
constexpr std::size_t kMaxChunkQueue = 8;
constexpr std::size_t kLatestFrameQueueDepth = 1;
constexpr uint32_t kRdImageWidth = 1024;
constexpr uint32_t kAngleFftSize = 64;
constexpr std::size_t kMaxPointDetections = 32;
constexpr float kLightSpeedMps = 3.0e8f;
constexpr float kPowerFloor = 1.0e-6f;

struct RawChunk {
    std::vector<uint8_t> data;
    rclcpp::Time stamp;
};

struct FramePacket {
    std::vector<uint8_t> bytes;
    rclcpp::Time stamp;
    uint32_t frame_index{0};
};

struct Detection {
    int range_bin{0};
    int doppler_bin{0};
    float power{0.0f};
};

struct PointDetection {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float velocity{0.0f};
    float intensity{0.0f};
};

struct ProcessedFrame {
    uint32_t frame_index{0};
    std::vector<uint8_t> rd_map_rgb;
    uint32_t rd_map_width{0};
    uint32_t rd_map_height{0};
    bool has_rd_map{false};
    bool has_point_cloud{false};
    std::vector<PointDetection> points;
};

std::size_t next_power_of_two(std::size_t value) {
    std::size_t power = 1;
    while (power < value) {
        power <<= 1;
    }
    return power;
}

void fft_inplace(std::vector<std::complex<float>>& data) {
    const std::size_t n = data.size();
    if (n <= 1) {
        return;
    }

    std::size_t j = 0;
    for (std::size_t i = 1; i < n; ++i) {
        std::size_t bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            std::swap(data[i], data[j]);
        }
    }

    for (std::size_t len = 2; len <= n; len <<= 1) {
        const float angle = -2.0f * static_cast<float>(M_PI) / static_cast<float>(len);
        const std::complex<float> wlen(std::cos(angle), std::sin(angle));
        for (std::size_t start = 0; start < n; start += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (std::size_t offset = 0; offset < len / 2; ++offset) {
                const auto u = data[start + offset];
                const auto v = data[start + offset + len / 2] * w;
                data[start + offset] = u + v;
                data[start + offset + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

template <typename T>
T clamp_value(const T value, const T low, const T high) {
    return std::max(low, std::min(value, high));
}

std::pair<bool, std::vector<uint8_t>> initialize_first_valid_packet(const uint32_t packet_sequence,
                                                                    const std::size_t bytes_per_frame,
                                                                    const uint8_t* packet_payload) {
    const uint64_t total_bytes = static_cast<uint64_t>(packet_sequence) * kPacketPayloadBytes;
    const uint64_t invalid_bytes = (total_bytes / bytes_per_frame) * bytes_per_frame;
    const uint64_t left_bytes = total_bytes - invalid_bytes;
    if (0 < left_bytes && left_bytes <= kPacketPayloadBytes) {
        return {true, std::vector<uint8_t>(packet_payload + (kPacketPayloadBytes - left_bytes),
                                           packet_payload + kPacketPayloadBytes)};
    }
    return {false, {}};
}

std::size_t compute_bytes_per_frame(const ADC_PARAMS& adc_params) {
    return static_cast<std::size_t>(adc_params.chirps) *
           static_cast<std::size_t>(adc_params.rx) *
           static_cast<std::size_t>(adc_params.tx) *
           static_cast<std::size_t>(adc_params.IQ) *
           static_cast<std::size_t>(adc_params.samples) *
           static_cast<std::size_t>(adc_params.bytes);
}

std::size_t range_fft_index(const int rx_idx,
                            const int chirp_idx,
                            const int range_bin,
                            const int chirp_count,
                            const int range_bin_count) {
    return (static_cast<std::size_t>(rx_idx) * chirp_count + static_cast<std::size_t>(chirp_idx)) *
               static_cast<std::size_t>(range_bin_count) +
           static_cast<std::size_t>(range_bin);
}

std::size_t rd_power_index(const int range_bin,
                           const int doppler_bin,
                           const int doppler_bin_count) {
    return static_cast<std::size_t>(range_bin) * static_cast<std::size_t>(doppler_bin_count) +
           static_cast<std::size_t>(doppler_bin);
}

void jet_color(const uint8_t value, uint8_t& red, uint8_t& green, uint8_t& blue) {
    const float v = static_cast<float>(value) / 255.0f;
    const float r = clamp_value(std::min(4.0f * v - 1.5f, -4.0f * v + 4.5f), 0.0f, 1.0f);
    const float g = clamp_value(std::min(4.0f * v - 0.5f, -4.0f * v + 3.5f), 0.0f, 1.0f);
    const float b = clamp_value(std::min(4.0f * v + 0.5f, -4.0f * v + 2.5f), 0.0f, 1.0f);
    red = static_cast<uint8_t>(255.0f * r);
    green = static_cast<uint8_t>(255.0f * g);
    blue = static_cast<uint8_t>(255.0f * b);
}

class RadarProcessorCore {
public:
    explicit RadarProcessorCore(const ADC_PARAMS& adc_params)
        : adc_params_(adc_params),
          bytes_per_frame_(compute_bytes_per_frame(adc_params)),
          range_fft_size_(static_cast<int>(next_power_of_two(static_cast<std::size_t>(adc_params.samples)))),
          doppler_fft_size_(static_cast<int>(next_power_of_two(static_cast<std::size_t>(adc_params.chirps)))) {
        frame_buffer_.reserve(bytes_per_frame_ + kPacketPayloadBytes);
    }

    std::size_t bytes_per_frame() const noexcept { return bytes_per_frame_; }

    void request_resync() {
        first_valid_packet_initialized_ = false;
        frame_buffer_.clear();
        expected_next_sequence_ = 0;
    }

    std::optional<FramePacket> ingest_chunk(const RawChunk& chunk) {
        if (chunk.data.empty()) {
            return std::nullopt;
        }
        if (chunk.data.size() % kPacketSize != 0) {
            request_resync();
            throw std::runtime_error("Received chunk size is not divisible by packet size");
        }

        const uint32_t first_sequence = *reinterpret_cast<const uint32_t*>(chunk.data.data());
        const uint32_t last_sequence =
            *reinterpret_cast<const uint32_t*>(chunk.data.data() + chunk.data.size() - kPacketSize);

        if (expected_next_sequence_ != 0 && first_sequence != expected_next_sequence_) {
            request_resync();
        }
        expected_next_sequence_ = last_sequence + 1;

        const std::size_t packet_count = chunk.data.size() / kPacketSize;
        for (std::size_t packet_index = 0; packet_index < packet_count; ++packet_index) {
            const uint8_t* raw_packet = chunk.data.data() + packet_index * kPacketSize;
            const uint32_t packet_sequence = *reinterpret_cast<const uint32_t*>(raw_packet);
            const uint8_t* packet_payload = raw_packet + kPacketHeaderBytes;

            if (!first_valid_packet_initialized_) {
                auto [is_valid, aligned_payload] =
                    initialize_first_valid_packet(packet_sequence, bytes_per_frame_, packet_payload);
                if (!is_valid) {
                    continue;
                }
                frame_buffer_.insert(frame_buffer_.end(), aligned_payload.begin(), aligned_payload.end());
                first_valid_packet_initialized_ = true;
            } else {
                frame_buffer_.insert(frame_buffer_.end(), packet_payload, packet_payload + kPacketPayloadBytes);
            }

            if (frame_buffer_.size() >= bytes_per_frame_) {
                FramePacket frame;
                frame.frame_index = frame_index_++;
                frame.stamp = chunk.stamp;
                frame.bytes.assign(frame_buffer_.begin(), frame_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_per_frame_));
                if (frame_buffer_.size() == bytes_per_frame_) {
                    frame_buffer_.clear();
                } else {
                    std::vector<uint8_t> remaining(frame_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_per_frame_),
                                                   frame_buffer_.end());
                    frame_buffer_.swap(remaining);
                }
                return frame;
            }
        }

        return std::nullopt;
    }

    ProcessedFrame process_frame(const FramePacket& frame,
                                 const bool build_rd_map,
                                 const bool build_point_cloud_enabled) const {
        const auto range_fft_all = compute_range_fft(frame.bytes);

        ProcessedFrame processed;
        processed.frame_index = frame.frame_index;
        const auto rd_power = compute_rd_power(range_fft_all);
        if (build_rd_map) {
            processed.has_rd_map = true;
            processed.rd_map_height = static_cast<uint32_t>(range_fft_size_);
            processed.rd_map_width = kRdImageWidth;
            processed.rd_map_rgb = build_rd_map_image(rd_power, processed.rd_map_width, processed.rd_map_height);
        }

        if (build_point_cloud_enabled) {
            const auto detections = detect_points(rd_power);
            processed.has_point_cloud = true;
            processed.points = build_point_cloud(range_fft_all, detections);
        }
        return processed;
    }

private:
    std::vector<std::complex<float>> compute_range_fft(const std::vector<uint8_t>& frame_bytes) const {
        const auto* raw_samples = reinterpret_cast<const int16_t*>(frame_bytes.data());
        const int total_chirps = adc_params_.chirps * adc_params_.tx;
        std::vector<std::complex<float>> range_fft_all(
            static_cast<std::size_t>(adc_params_.rx) *
            static_cast<std::size_t>(adc_params_.chirps) *
            static_cast<std::size_t>(range_fft_size_));

        std::vector<std::complex<float>> fft_input(range_fft_size_, std::complex<float>(0.0f, 0.0f));

        for (int chirp_linear = 0; chirp_linear < total_chirps; ++chirp_linear) {
            const int tx_index = chirp_linear % adc_params_.tx;
            if (tx_index != 0) {
                continue;
            }
            const int chirp_index = chirp_linear / adc_params_.tx;
            for (int rx_index = 0; rx_index < adc_params_.rx; ++rx_index) {
                std::fill(fft_input.begin(), fft_input.end(), std::complex<float>(0.0f, 0.0f));
                for (int sample_index = 0; sample_index < adc_params_.samples; ++sample_index) {
                    const std::size_t base_index =
                        ((((static_cast<std::size_t>(chirp_linear) * static_cast<std::size_t>(adc_params_.samples)) +
                            static_cast<std::size_t>(sample_index)) *
                               static_cast<std::size_t>(adc_params_.IQ)) *
                              static_cast<std::size_t>(adc_params_.rx)) +
                         static_cast<std::size_t>(rx_index);

                    if (adc_params_.IQ == 2) {
                        const float i_value = static_cast<float>(raw_samples[base_index]);
                        const float q_value = static_cast<float>(raw_samples[base_index + static_cast<std::size_t>(adc_params_.rx)]);
                        fft_input[sample_index] = std::complex<float>(i_value, q_value);
                    } else {
                        fft_input[sample_index] = std::complex<float>(static_cast<float>(raw_samples[base_index]), 0.0f);
                    }
                }

                fft_inplace(fft_input);
                for (int range_bin = 0; range_bin < range_fft_size_; ++range_bin) {
                    range_fft_all[range_fft_index(rx_index, chirp_index, range_bin, adc_params_.chirps, range_fft_size_)] =
                        fft_input[range_bin];
                }
            }
        }

        return range_fft_all;
    }

    std::vector<float> compute_rd_power(const std::vector<std::complex<float>>& range_fft_all) const {
        std::vector<float> rd_power(static_cast<std::size_t>(range_fft_size_) *
                                        static_cast<std::size_t>(doppler_fft_size_),
                                    0.0f);
        std::vector<std::complex<float>> doppler_input(doppler_fft_size_, std::complex<float>(0.0f, 0.0f));

        for (int range_bin = 0; range_bin < range_fft_size_; ++range_bin) {
            std::fill(doppler_input.begin(), doppler_input.end(), std::complex<float>(0.0f, 0.0f));
            for (int chirp_index = 0; chirp_index < adc_params_.chirps; ++chirp_index) {
                doppler_input[chirp_index] =
                    range_fft_all[range_fft_index(0, chirp_index, range_bin, adc_params_.chirps, range_fft_size_)];
            }

            fft_inplace(doppler_input);
            const int half = doppler_fft_size_ / 2;
            for (int doppler_bin = 0; doppler_bin < doppler_fft_size_; ++doppler_bin) {
                const int shifted_index = (doppler_bin + half) % doppler_fft_size_;
                const float power = std::norm(doppler_input[doppler_bin]);
                rd_power[rd_power_index(range_bin, shifted_index, doppler_fft_size_)] = power;
            }
        }

        return rd_power;
    }

    std::vector<Detection> detect_points(const std::vector<float>& rd_power) const {
        constexpr int training_range = 4;
        constexpr int training_doppler = 8;
        constexpr int guard_range = 1;
        constexpr int guard_doppler = 2;
        constexpr float threshold_scale = 4.5f;

        const int rows = range_fft_size_;
        const int cols = doppler_fft_size_;
        std::vector<double> integral(static_cast<std::size_t>(rows + 1) * static_cast<std::size_t>(cols + 1), 0.0);

        for (int row = 0; row < rows; ++row) {
            double row_sum = 0.0;
            for (int col = 0; col < cols; ++col) {
                row_sum += rd_power[rd_power_index(row, col, cols)];
                integral[static_cast<std::size_t>(row + 1) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(col + 1)] =
                    integral[static_cast<std::size_t>(row) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(col + 1)] +
                    row_sum;
            }
        }

        const auto rect_sum = [&](int r0, int c0, int r1, int c1) -> double {
            return integral[static_cast<std::size_t>(r1 + 1) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(c1 + 1)] -
                   integral[static_cast<std::size_t>(r0) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(c1 + 1)] -
                   integral[static_cast<std::size_t>(r1 + 1) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(c0)] +
                   integral[static_cast<std::size_t>(r0) * static_cast<std::size_t>(cols + 1) + static_cast<std::size_t>(c0)];
        };

        const int outer_range = training_range + guard_range;
        const int outer_doppler = training_doppler + guard_doppler;
        const int outer_cells = (2 * outer_range + 1) * (2 * outer_doppler + 1);
        const int inner_cells = (2 * guard_range + 1) * (2 * guard_doppler + 1);
        const int training_cells = outer_cells - inner_cells;

        std::vector<Detection> detections;
        detections.reserve(kMaxPointDetections * 2);

        for (int row = outer_range; row < rows - outer_range; ++row) {
            for (int col = outer_doppler; col < cols - outer_doppler; ++col) {
                const double outer_sum = rect_sum(row - outer_range, col - outer_doppler, row + outer_range, col + outer_doppler);
                const double inner_sum = rect_sum(row - guard_range, col - guard_doppler, row + guard_range, col + guard_doppler);
                const double training_average = (outer_sum - inner_sum) / static_cast<double>(training_cells);
                const float cell_power = rd_power[rd_power_index(row, col, cols)];
                if (cell_power <= static_cast<float>(training_average * threshold_scale)) {
                    continue;
                }

                bool local_maximum = true;
                for (int row_offset = -1; row_offset <= 1 && local_maximum; ++row_offset) {
                    for (int col_offset = -1; col_offset <= 1; ++col_offset) {
                        if (row_offset == 0 && col_offset == 0) {
                            continue;
                        }
                        if (rd_power[rd_power_index(row + row_offset, col + col_offset, cols)] >= cell_power) {
                            local_maximum = false;
                            break;
                        }
                    }
                }

                if (local_maximum) {
                    detections.push_back({row, col, cell_power});
                }
            }
        }

        std::sort(detections.begin(), detections.end(), [](const Detection& lhs, const Detection& rhs) {
            return lhs.power > rhs.power;
        });
        if (detections.size() > kMaxPointDetections) {
            detections.resize(kMaxPointDetections);
        }
        return detections;
    }

    std::vector<PointDetection> build_point_cloud(const std::vector<std::complex<float>>& range_fft_all,
                                                  const std::vector<Detection>& detections) const {
        const float adc_sample_time_us =
            1.0e3f * static_cast<float>(adc_params_.samples) / static_cast<float>(adc_params_.sample_rate);
        const float mid_freq_hz =
            static_cast<float>(adc_params_.startFreq) * 1.0e9f +
            static_cast<float>(adc_params_.adc_valid_start_time + adc_sample_time_us / 2.0f) *
                static_cast<float>(adc_params_.freq_slope) * 1.0e6f;
        const float chirp_repetition_period_us =
            static_cast<float>(adc_params_.tx) *
            static_cast<float>(adc_params_.idleTime + adc_params_.rampEndTime);
        const float wavelength_m = kLightSpeedMps / mid_freq_hz;
        const float max_velocity = wavelength_m / (4.0f * chirp_repetition_period_us * 1.0e-6f);
        const float max_range =
            (300.0f * 0.8f * static_cast<float>(adc_params_.sample_rate)) /
            (2.0f * static_cast<float>(adc_params_.freq_slope) * 1.0e3f);

        std::set<int> unique_range_bins;
        for (const auto& detection : detections) {
            unique_range_bins.insert(detection.range_bin);
        }

        std::map<int, std::vector<std::vector<std::complex<float>>>> doppler_cache;
        std::vector<std::complex<float>> doppler_input(doppler_fft_size_, std::complex<float>(0.0f, 0.0f));

        for (const int range_bin : unique_range_bins) {
            auto& per_rx_spectra = doppler_cache[range_bin];
            per_rx_spectra.resize(static_cast<std::size_t>(adc_params_.rx));
            for (int rx_index = 0; rx_index < adc_params_.rx; ++rx_index) {
                std::fill(doppler_input.begin(), doppler_input.end(), std::complex<float>(0.0f, 0.0f));
                for (int chirp_index = 0; chirp_index < adc_params_.chirps; ++chirp_index) {
                    doppler_input[chirp_index] =
                        range_fft_all[range_fft_index(rx_index, chirp_index, range_bin, adc_params_.chirps, range_fft_size_)];
                }
                fft_inplace(doppler_input);
                auto& spectrum = per_rx_spectra[static_cast<std::size_t>(rx_index)];
                spectrum.resize(static_cast<std::size_t>(doppler_fft_size_));
                const int half = doppler_fft_size_ / 2;
                for (int doppler_bin = 0; doppler_bin < doppler_fft_size_; ++doppler_bin) {
                    const int shifted_index = (doppler_bin + half) % doppler_fft_size_;
                    spectrum[static_cast<std::size_t>(shifted_index)] = doppler_input[doppler_bin];
                }
            }
        }

        std::vector<PointDetection> points;
        points.reserve(detections.size());
        for (const auto& detection : detections) {
            std::vector<std::complex<float>> angle_input(kAngleFftSize, std::complex<float>(0.0f, 0.0f));
            const auto& per_rx_spectra = doppler_cache[detection.range_bin];
            for (int rx_index = 0; rx_index < adc_params_.rx; ++rx_index) {
                angle_input[static_cast<std::size_t>(rx_index)] =
                    per_rx_spectra[static_cast<std::size_t>(rx_index)][static_cast<std::size_t>(detection.doppler_bin)];
            }
            fft_inplace(angle_input);

            const int angle_half = static_cast<int>(kAngleFftSize / 2);
            float best_magnitude = -1.0f;
            int best_index = 0;
            for (int index = 0; index < static_cast<int>(kAngleFftSize); ++index) {
                const int shifted_index = (index + angle_half) % static_cast<int>(kAngleFftSize);
                const float magnitude = std::abs(angle_input[static_cast<std::size_t>(index)]);
                if (magnitude > best_magnitude) {
                    best_magnitude = magnitude;
                    best_index = shifted_index;
                }
            }

            const float normalized_spatial =
                clamp_value(2.0f * (static_cast<float>(best_index) - static_cast<float>(angle_half)) /
                                static_cast<float>(kAngleFftSize),
                            -1.0f,
                            1.0f);
            const float azimuth_rad = std::asin(normalized_spatial);
            const float range_m = (static_cast<float>(detection.range_bin) / static_cast<float>(range_fft_size_)) * max_range;
            const float velocity_mps =
                ((static_cast<float>(detection.doppler_bin) - static_cast<float>(doppler_fft_size_ / 2)) /
                 static_cast<float>(doppler_fft_size_)) *
                2.0f * max_velocity;

            points.push_back({
                range_m * std::cos(azimuth_rad),
                range_m * std::sin(azimuth_rad),
                0.0f,
                velocity_mps,
                10.0f * std::log10(std::max(detection.power, kPowerFloor)),
            });
        }

        return points;
    }

    std::vector<uint8_t> build_rd_map_image(const std::vector<float>& rd_power,
                                            const uint32_t output_width,
                                            const uint32_t output_height) const {
        const int rows = range_fft_size_;
        const int cols = doppler_fft_size_;
        std::vector<float> rd_db(rd_power.size(), 0.0f);
        float max_db = -std::numeric_limits<float>::infinity();
        for (std::size_t index = 0; index < rd_power.size(); ++index) {
            rd_db[index] = 10.0f * std::log10(std::max(rd_power[index], kPowerFloor));
            max_db = std::max(max_db, rd_db[index]);
        }
        const float min_db = max_db - 50.0f;

        std::vector<uint8_t> image(static_cast<std::size_t>(output_width) *
                                       static_cast<std::size_t>(output_height) * 3,
                                   0);

        for (uint32_t out_row = 0; out_row < output_height; ++out_row) {
            const int row_start = static_cast<int>((static_cast<double>(out_row) / output_height) * rows);
            const int row_end = std::max(row_start + 1,
                                         static_cast<int>((static_cast<double>(out_row + 1) / output_height) * rows));
            for (uint32_t out_col = 0; out_col < output_width; ++out_col) {
                const int col_start = static_cast<int>((static_cast<double>(out_col) / output_width) * cols);
                const int col_end = std::max(col_start + 1,
                                             static_cast<int>((static_cast<double>(out_col + 1) / output_width) * cols));

                float pooled_db = min_db;
                for (int row = row_start; row < row_end; ++row) {
                    for (int col = col_start; col < col_end; ++col) {
                        pooled_db = std::max(pooled_db, rd_db[rd_power_index(row, col, cols)]);
                    }
                }

                const float normalized = clamp_value((pooled_db - min_db) / (max_db - min_db + 1e-6f), 0.0f, 1.0f);
                const uint8_t level = static_cast<uint8_t>(normalized * 255.0f);
                uint8_t red = 0;
                uint8_t green = 0;
                uint8_t blue = 0;
                jet_color(level, red, green, blue);

                const std::size_t image_index =
                    (static_cast<std::size_t>(output_height - 1 - out_row) * static_cast<std::size_t>(output_width) +
                     static_cast<std::size_t>(out_col)) * 3;
                image[image_index] = red;
                image[image_index + 1] = green;
                image[image_index + 2] = blue;
            }
        }

        return image;
    }

    ADC_PARAMS adc_params_;
    std::size_t bytes_per_frame_{0};
    int range_fft_size_{0};
    int doppler_fft_size_{0};
    bool first_valid_packet_initialized_{false};
    uint32_t expected_next_sequence_{0};
    uint32_t frame_index_{0};
    std::vector<uint8_t> frame_buffer_;
};

class RadarProcessorNode final : public rclcpp::Node {
public:
    RadarProcessorNode()
        : Node("radar_processor_cpp"),
          stop_requested_(false) {
        declare_parameter<std::string>("radar_config_filename", kDefaultRadarConfigFilename);
        declare_parameter<int>("rd_every_n_frames", 1);
        declare_parameter<int>("pointcloud_every_n_frames", 20);
        get_parameter("radar_config_filename", radar_config_filename_);
        rd_every_n_frames_ = std::max<int64_t>(1, get_parameter("rd_every_n_frames").as_int());
        pointcloud_every_n_frames_ = std::max<int64_t>(1, get_parameter("pointcloud_every_n_frames").as_int());

        const auto publisher_config = radar_driver_cpp::core::load_publisher_config(
            radar_config_filename_, "dca_config.txt");
        adc_params_ = awr2243_read_config(publisher_config.radar_config_path);
        processor_core_ = std::make_unique<RadarProcessorCore>(adc_params_);

        auto subscription_qos = rclcpp::QoS(rclcpp::KeepLast(4));
        subscription_qos.best_effort();
        subscription_qos.durability_volatile();

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>(kRdMapTopic, rclcpp::QoS(2).best_effort());
        pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kPointCloudTopic, rclcpp::QoS(2).best_effort());

        chunk_subscription_ = create_subscription<my_msgs::msg::RawRadar>(
            kRadarInputTopic,
            subscription_qos,
            [this](my_msgs::msg::RawRadar::UniquePtr msg) {
                on_chunk(std::move(msg));
            });

        assembler_thread_ = std::thread(&RadarProcessorNode::run_assembler_loop, this);
        processing_thread_ = std::thread(&RadarProcessorNode::run_processing_loop, this);
        RCLCPP_INFO(get_logger(),
                    "Radar processor started with config %s (rd_every_n_frames=%ld, pointcloud_every_n_frames=%ld)",
                    radar_config_filename_.c_str(),
                    rd_every_n_frames_,
                    pointcloud_every_n_frames_);
    }

    ~RadarProcessorNode() override {
        stop_requested_ = true;
        chunk_condition_.notify_all();
        frame_condition_.notify_all();
        if (assembler_thread_.joinable()) {
            assembler_thread_.join();
        }
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    void on_chunk(my_msgs::msg::RawRadar::UniquePtr msg) {
        RawChunk chunk;
        chunk.data = std::move(msg->data);
        chunk.stamp = now();

        {
            std::lock_guard<std::mutex> lock(chunk_mutex_);
            if (chunk_queue_.size() >= kMaxChunkQueue) {
                chunk_queue_.clear();
                resync_requested_ = true;
                ++dropped_chunk_batches_;
            }
            chunk_queue_.emplace_back(std::move(chunk));
        }
        chunk_condition_.notify_one();
    }

    void run_assembler_loop() {
        while (!stop_requested_) {
            RawChunk chunk;
            {
                std::unique_lock<std::mutex> lock(chunk_mutex_);
                chunk_condition_.wait(lock, [&]() { return stop_requested_ || !chunk_queue_.empty(); });
                if (stop_requested_ && chunk_queue_.empty()) {
                    return;
                }
                chunk = std::move(chunk_queue_.front());
                chunk_queue_.pop_front();
            }

            try {
                if (resync_requested_) {
                    processor_core_->request_resync();
                    resync_requested_ = false;
                    RCLCPP_WARN(get_logger(), "processor resynchronized after chunk backlog or loss");
                }

                const auto frame = processor_core_->ingest_chunk(chunk);
                if (!frame.has_value()) {
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    if (pending_frames_.size() >= kLatestFrameQueueDepth) {
                        pending_frames_.clear();
                        ++dropped_processed_frames_;
                    }
                    pending_frames_.emplace_back(*frame);
                }
                frame_condition_.notify_one();
            } catch (const std::exception& error) {
                processor_core_->request_resync();
                RCLCPP_WARN(get_logger(), "processor chunk assembly reset: %s", error.what());
            }
        }
    }

    void run_processing_loop() {
        while (!stop_requested_) {
            FramePacket frame;
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                frame_condition_.wait(lock, [&]() { return stop_requested_ || !pending_frames_.empty(); });
                if (stop_requested_ && pending_frames_.empty()) {
                    return;
                }
                frame = std::move(pending_frames_.back());
                pending_frames_.clear();
            }

            try {
                const bool build_rd_map = (frame.frame_index % static_cast<uint32_t>(rd_every_n_frames_)) == 0U;
                const bool build_point_cloud_enabled =
                    (frame.frame_index % static_cast<uint32_t>(pointcloud_every_n_frames_)) == 0U;
                const auto processed =
                    processor_core_->process_frame(frame, build_rd_map, build_point_cloud_enabled);
                if (processed.has_rd_map) {
                    publish_rd_map(processed, frame.stamp);
                }
                if (processed.has_point_cloud) {
                    publish_point_cloud(processed, frame.stamp);
                }
                if ((processed.frame_index % 10U) == 0U) {
                    RCLCPP_INFO(get_logger(),
                                "[config=%s] processed frame=%u rd=%s pointcloud=%s points=%zu",
                                radar_config_filename_.c_str(),
                                processed.frame_index,
                                processed.has_rd_map ? "yes" : "no",
                                processed.has_point_cloud ? "yes" : "no",
                                processed.points.size());
                }
            } catch (const std::exception& error) {
                RCLCPP_WARN(get_logger(), "processor frame dropped: %s", error.what());
            }
        }
    }

    void publish_rd_map(const ProcessedFrame& frame, const rclcpp::Time& stamp) {
        sensor_msgs::msg::Image image_msg;
        image_msg.header.stamp = stamp;
        image_msg.header.frame_id = "radar";
        image_msg.height = frame.rd_map_height;
        image_msg.width = frame.rd_map_width;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = frame.rd_map_width * 3;
        image_msg.data = frame.rd_map_rgb;
        image_publisher_->publish(image_msg);
    }

    void publish_point_cloud(const ProcessedFrame& frame, const rclcpp::Time& stamp) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = "radar";
        cloud_msg.height = 1;
        cloud_msg.width = static_cast<uint32_t>(frame.points.size());
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(frame.points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_velocity(cloud_msg, "velocity");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

        for (const auto& point : frame.points) {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_velocity = point.velocity;
            *iter_intensity = point.intensity;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_velocity;
            ++iter_intensity;
        }

        pointcloud_publisher_->publish(cloud_msg);
    }

    rclcpp::Subscription<my_msgs::msg::RawRadar>::SharedPtr chunk_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    ADC_PARAMS adc_params_{};
    std::unique_ptr<RadarProcessorCore> processor_core_;
    std::string radar_config_filename_;
    int64_t rd_every_n_frames_{1};
    int64_t pointcloud_every_n_frames_{20};

    std::atomic<bool> stop_requested_;
    std::atomic<uint64_t> dropped_chunk_batches_{0};
    std::atomic<uint64_t> dropped_processed_frames_{0};
    std::atomic<bool> resync_requested_{false};

    std::mutex chunk_mutex_;
    std::condition_variable chunk_condition_;
    std::deque<RawChunk> chunk_queue_;

    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    std::deque<FramePacket> pending_frames_;

    std::thread assembler_thread_;
    std::thread processing_thread_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
