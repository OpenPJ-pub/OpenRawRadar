#include "capture_app.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cstring>

#include "capture_config.h"
#include "capture_session.h"
#include "platform/output_owner.h"

namespace fs = std::filesystem;

namespace open_raw_radar::cli {
namespace {

constexpr uint32_t kPacketsPerFetch = 500;
constexpr std::size_t kBytesPerPacket = 1466;
constexpr std::size_t kPacketPayloadBytes = 1456;
constexpr std::size_t kMaxPendingChunks = 64;
constexpr const char* kDefaultRadarConfigFilename = "AWR2243_mmwaveconfig_max15.txt";
constexpr const char* kDefaultDcaConfigFilename = "dca_config.txt";
constexpr const char* kLoggerName = "open_raw_radar_capture";

std::atomic<bool> g_stop_requested{false};

void handle_signal(int) {
    g_stop_requested = true;
}

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

std::string build_default_output_dir() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    std::ostringstream stream;
    stream << "cli_" << std::put_time(&local_tm, "%Y_%m_%d-%H_%M_%S");
    return (fs::path(OPEN_RAW_RADAR_REPO_ROOT) / "output" / stream.str()).string();
}

std::string escape_json(const std::string& input) {
    std::ostringstream stream;
    for (const char ch : input) {
        switch (ch) {
            case '\\': stream << "\\\\"; break;
            case '"': stream << "\\\""; break;
            case '\n': stream << "\\n"; break;
            case '\r': stream << "\\r"; break;
            case '\t': stream << "\\t"; break;
            default: stream << ch; break;
        }
    }
    return stream.str();
}

std::string trigger_mode_to_string(const TriggerMode mode) {
    return mode == TriggerMode::kHardware ? "hardware" : "software";
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

struct CliOptions {
    std::string radar_config_filename{kDefaultRadarConfigFilename};
    std::string dca_config_filename{kDefaultDcaConfigFilename};
    std::string output_dir{build_default_output_dir()};
    std::string subnet;
    std::string host_suffix;
    std::string board_suffix;
    uint64_t max_frames{0};
};

CliOptions parse_cli_options(int argc, char* argv[]) {
    CliOptions options;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        const auto require_value = [&](const std::string& flag) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            return argv[++index];
        };

        if (arg == "--radar-config-filename") {
            options.radar_config_filename = require_value(arg);
        } else if (arg == "--dca-config-filename") {
            options.dca_config_filename = require_value(arg);
        } else if (arg == "--output-dir") {
            options.output_dir = require_value(arg);
        } else if (arg == "--subnet") {
            options.subnet = require_value(arg);
        } else if (arg == "--host-suffix") {
            options.host_suffix = require_value(arg);
        } else if (arg == "--board-suffix") {
            options.board_suffix = require_value(arg);
        } else if (arg == "--max-frames") {
            options.max_frames = std::stoull(require_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: open_raw_radar_capture [--radar-config-filename FILE] "
                << "[--dca-config-filename FILE] [--output-dir DIR] [--subnet N] "
                << "[--host-suffix N] [--board-suffix N] [--max-frames N]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    return options;
}

struct CaptureStatistics {
    uint64_t chunk_count{0};
    uint64_t packet_count{0};
    uint64_t frame_count{0};
    uint64_t bytes_written{0};
    uint64_t discarded_tail_bytes{0};
    uint32_t first_sequence{0};
    uint32_t last_sequence{0};
    std::string first_chunk_ts;
    std::string last_chunk_ts;
    std::string stop_reason{"completed"};
};

class CaptureLogger {
public:
    explicit CaptureLogger(const fs::path& log_path)
        : log_file_(log_path, std::ios::out | std::ios::trunc) {
        if (!log_file_.is_open()) {
            throw std::runtime_error("Unable to open log file: " + log_path.string());
        }
    }

    void info(const std::string& message) {
        write("INFO", message);
    }

    void error(const std::string& message) {
        write("ERROR", message);
    }

private:
    void write(const std::string& level, const std::string& message) {
        const std::string ts = format_local_timestamp();
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "[" << level << "] "
                  << "[" << ts << "] "
                  << "[" << kLoggerName << "]: "
                  << message
                  << std::endl;
        log_file_ << "[" << level << "] "
                  << "[" << ts << "] "
                  << "[" << kLoggerName << "]: "
                  << message
                  << '\n';
        log_file_.flush();
    }

    std::mutex mutex_;
    std::ofstream log_file_;
};

class AsyncChunkWriter {
public:
    AsyncChunkWriter(fs::path output_path,
                     CaptureLogger& logger,
                     std::size_t bytes_per_frame)
        : output_path_(std::move(output_path)),
          logger_(logger),
          bytes_per_frame_(bytes_per_frame) {}

    void start() {
        output_file_.open(output_path_, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!output_file_.is_open()) {
            throw std::runtime_error("Unable to open output file: " + output_path_.string());
        }
        worker_ = std::thread(&AsyncChunkWriter::run, this);
    }

    void enqueue(std::vector<uint8_t>&& data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= kMaxPendingChunks) {
                throw std::runtime_error("Writer queue backlog exceeded safe limit");
            }
            queue_.emplace_back(std::move(data));
        }
        condition_.notify_one();
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
        output_file_.flush();
        output_file_.close();
    }

    uint64_t bytes_written() const noexcept {
        return bytes_written_.load();
    }

    uint64_t frames_written() const noexcept {
        return frames_written_.load();
    }

    uint64_t discarded_tail_bytes() const noexcept {
        return discarded_tail_bytes_.load();
    }

    bool failed() const noexcept {
        return failed_.load();
    }

    std::string error_message() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_message_;
    }

private:
    void run() {
        try {
            while (true) {
                std::vector<uint8_t> data;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                    if (queue_.empty()) {
                        if (stopping_) {
                            break;
                        }
                        continue;
                    }
                    data = std::move(queue_.front());
                    queue_.pop_front();
                }
                append_chunk(std::move(data));
            }
            discarded_tail_bytes_ = frame_buffer_.size();
        } catch (const std::exception& error) {
            failed_ = true;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                error_message_ = error.what();
            }
            logger_.error(std::string("writer thread failed: ") + error.what());
        }
    }

    void append_chunk(std::vector<uint8_t>&& data) {
        if (data.size() % kBytesPerPacket != 0) {
            throw std::runtime_error("Capture chunk size is not divisible by packet size");
        }

        const std::size_t packet_count = data.size() / kBytesPerPacket;
        for (std::size_t packet_index = 0; packet_index < packet_count; ++packet_index) {
            const uint8_t* raw_packet = data.data() + packet_index * kBytesPerPacket;
            const uint32_t packet_sequence = *reinterpret_cast<const uint32_t*>(raw_packet);
            const uint8_t* packet_payload = raw_packet + 10;

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

            while (frame_buffer_.size() >= bytes_per_frame_) {
                output_file_.write(reinterpret_cast<const char*>(frame_buffer_.data()),
                                   static_cast<std::streamsize>(bytes_per_frame_));
                if (!output_file_) {
                    throw std::runtime_error("Failed to write capture frame to output file");
                }
                bytes_written_ += static_cast<uint64_t>(bytes_per_frame_);
                frames_written_ += 1;
                frame_buffer_.erase(frame_buffer_.begin(), frame_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_per_frame_));
            }
        }
    }

    fs::path output_path_;
    CaptureLogger& logger_;
    std::size_t bytes_per_frame_{0};
    std::ofstream output_file_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<std::vector<uint8_t>> queue_;
    std::atomic<uint64_t> bytes_written_{0};
    std::atomic<uint64_t> frames_written_{0};
    std::atomic<uint64_t> discarded_tail_bytes_{0};
    std::atomic<bool> failed_{false};
    bool stopping_{false};
    std::string error_message_;
    bool first_valid_packet_initialized_{false};
    std::vector<uint8_t> frame_buffer_;
};

class CaptureRecorder {
public:
    CaptureRecorder(CaptureLogger& logger, std::string radar_config_filename)
        : logger_(logger),
          radar_config_filename_(std::move(radar_config_filename)) {}

    void record_chunk(const PacketChunk& chunk,
                      const std::size_t packets_per_frame,
                      CaptureStatistics& stats) {
        stats.chunk_count += 1;
        stats.packet_count += kPacketsPerFetch;
        if (stats.first_sequence == 0) {
            stats.first_sequence = chunk.first_sequence;
            stats.first_chunk_ts = format_local_timestamp();
        }
        stats.last_sequence = chunk.last_sequence;
        stats.last_chunk_ts = format_local_timestamp();

        pending_packets_ += kPacketsPerFetch;
        while (pending_packets_ >= packets_per_frame) {
            pending_packets_ -= static_cast<uint64_t>(packets_per_frame);
            stats.frame_count += 1;
            logger_.info(
                "[config=" + radar_config_filename_ + "] frame=" + std::to_string(stats.frame_count) +
                " ts=" + format_local_timestamp());
        }
    }

private:
    CaptureLogger& logger_;
    std::string radar_config_filename_;
    uint64_t pending_packets_{0};
};

class StandaloneCaptureApp {
public:
    explicit StandaloneCaptureApp(CliOptions options)
        : options_(std::move(options)),
          output_dir_(options_.output_dir) {}

    int run() {
        install_signal_handlers();
        prepare_output_dir();

        CaptureLogger logger(output_dir_ / "capture.log");
        logger_ = &logger;
        logger_->info("loading capture configuration");

        std::unique_ptr<AsyncChunkWriter> writer;
        std::unique_ptr<CaptureSession> capture_session;
        bool writer_started = false;
        bool capture_prepared = false;

        try {
            config_ = load_capture_config(
                options_.radar_config_filename,
                options_.dca_config_filename,
                options_.subnet,
                options_.host_suffix,
                options_.board_suffix);

            auto radar_udp = std::make_shared<RadarUDP>(config_.network.board_ip,
                                                        config_.network.host_ip,
                                                        4096,
                                                        4098);
            capture_session = std::make_unique<CaptureSession>(
                std::move(radar_udp),
                config_.radar_config_path,
                config_.dca_config_path,
                config_.packets_per_frame);
            CaptureRecorder recorder(*logger_, config_.radar_config_filename);
            writer = std::make_unique<AsyncChunkWriter>(
                output_dir_ / "adc_data.bin",
                *logger_,
                config_.bytes_per_frame);

            logger_->info("Radar config: " + config_.radar_config_filename);
            logger_->info("DCA config: " + config_.dca_config_filename);
            logger_->info("Subnet: " + config_.network.subnet);
            logger_->info("Host IP: " + config_.network.host_ip);
            logger_->info("DCA1000 IP: " + config_.network.board_ip);

            writer->start();
            writer_started = true;
            logger_->info("starting radar capture");
            capture_session->prepare();
            capture_prepared = true;

            capture_loop(*capture_session, *writer, recorder);
            statistics_.bytes_written = writer->bytes_written();
            statistics_.frame_count = writer->frames_written();
            statistics_.discarded_tail_bytes = writer->discarded_tail_bytes();
            if (capture_prepared) {
                capture_session->shutdown();
            }
            if (writer_started) {
                writer->stop();
            }
            if (writer->failed()) {
                throw std::runtime_error(writer->error_message());
            }
            write_metadata();
            finalize_output_owner(output_dir_);
            logger_->info("capture completed");
            return 0;
        } catch (const std::exception& error) {
            statistics_.stop_reason = error.what();
            if (capture_prepared && capture_session) {
                try {
                    capture_session->shutdown();
                } catch (...) {
                }
            }
            if (writer_started && writer) {
                writer->stop();
            }
            if (writer) {
                statistics_.bytes_written = writer->bytes_written();
                statistics_.frame_count = writer->frames_written();
                statistics_.discarded_tail_bytes = writer->discarded_tail_bytes();
            }
            try {
                write_metadata();
            } catch (const std::exception& metadata_error) {
                logger_->error(std::string("failed to write metadata: ") + metadata_error.what());
            }
            try {
                finalize_output_owner(output_dir_);
            } catch (const std::exception& ownership_error) {
                logger_->error(std::string("failed to update output ownership: ") + ownership_error.what());
            }
            logger_->error(std::string("capture aborted: ") + error.what());
            return 1;
        }
    }

private:
    void install_signal_handlers() const {
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
    }

    void prepare_output_dir() const {
        fs::create_directories(output_dir_);
        prepare_output_owner(output_dir_);
    }

    void capture_loop(CaptureSession& capture_session,
                      AsyncChunkWriter& writer,
                      CaptureRecorder& recorder) {
        while (!g_stop_requested.load()) {
            if (writer.failed()) {
                throw std::runtime_error(writer.error_message());
            }

            auto chunk = capture_session.fetch_next_chunk();
            if (!chunk.has_data()) {
                continue;
            }

            recorder.record_chunk(chunk, config_.packets_per_frame, statistics_);
            writer.enqueue(std::move(chunk.data));

            if (options_.max_frames > 0 && statistics_.frame_count >= options_.max_frames) {
                statistics_.stop_reason = "max_frames_reached";
                break;
            }
        }

        if (g_stop_requested.load() && statistics_.stop_reason == "completed") {
            statistics_.stop_reason = "signal_stop";
        }
    }

    void write_metadata() const {
        const fs::path metadata_path = output_dir_ / "adc_data.json";
        std::ofstream metadata_file(metadata_path, std::ios::out | std::ios::trunc);
        if (!metadata_file.is_open()) {
            throw std::runtime_error("Unable to write metadata file: " + metadata_path.string());
        }

        metadata_file
            << "{\n"
            << "  \"radar_config_filename\": \"" << escape_json(config_.radar_config_filename) << "\",\n"
            << "  \"dca_config_filename\": \"" << escape_json(config_.dca_config_filename) << "\",\n"
            << "  \"radar_config_path\": \"" << escape_json(config_.radar_config_path) << "\",\n"
            << "  \"dca_config_path\": \"" << escape_json(config_.dca_config_path) << "\",\n"
            << "  \"output_dir\": \"" << escape_json(output_dir_.string()) << "\",\n"
            << "  \"trigger_mode\": \"" << trigger_mode_to_string(config_.trigger_mode) << "\",\n"
            << "  \"subnet\": \"" << escape_json(config_.network.subnet) << "\",\n"
            << "  \"host_suffix\": \"" << escape_json(config_.network.host_suffix) << "\",\n"
            << "  \"board_suffix\": \"" << escape_json(config_.network.board_suffix) << "\",\n"
            << "  \"board_ip\": \"" << escape_json(config_.network.board_ip) << "\",\n"
            << "  \"host_ip\": \"" << escape_json(config_.network.host_ip) << "\",\n"
            << "  \"bytes_per_frame\": " << config_.bytes_per_frame << ",\n"
            << "  \"packets_per_frame\": " << config_.packets_per_frame << ",\n"
            << "  \"chunk_size_packets\": " << kPacketsPerFetch << ",\n"
            << "  \"packet_size_bytes\": " << kBytesPerPacket << ",\n"
            << "  \"packet_payload_bytes\": " << kPacketPayloadBytes << ",\n"
            << "  \"bin_layout\": \"frame_aligned_adc_payload\",\n"
            << "  \"chunk_count\": " << statistics_.chunk_count << ",\n"
            << "  \"packet_count\": " << statistics_.packet_count << ",\n"
            << "  \"frame_count\": " << statistics_.frame_count << ",\n"
            << "  \"bytes_written\": " << statistics_.bytes_written << ",\n"
            << "  \"data_size_consistent\": true,\n"
            << "  \"discarded_tail_bytes\": " << statistics_.discarded_tail_bytes << ",\n"
            << "  \"first_sequence\": " << statistics_.first_sequence << ",\n"
            << "  \"last_sequence\": " << statistics_.last_sequence << ",\n"
            << "  \"first_chunk_ts\": \"" << escape_json(statistics_.first_chunk_ts) << "\",\n"
            << "  \"last_chunk_ts\": \"" << escape_json(statistics_.last_chunk_ts) << "\",\n"
            << "  \"stop_reason\": \"" << escape_json(statistics_.stop_reason) << "\"\n"
            << "}\n";
    }

    CliOptions options_;
    fs::path output_dir_;
    CaptureConfig config_{};
    CaptureStatistics statistics_{};
    CaptureLogger* logger_{nullptr};
};

}  // namespace

int run_capture_app(int argc, char* argv[]) {
    StandaloneCaptureApp app(parse_cli_options(argc, argv));
    return app.run();
}

}  // namespace open_raw_radar::cli
