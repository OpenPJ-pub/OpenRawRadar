#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "radar_driver_cpp/transport/ftdi_comm.h"

namespace {

constexpr const char* kLoggerName = "open_raw_radar_firmware_download";

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

void log_line(const std::string& level, const std::string& message) {
    std::cout << "[" << level << "] "
              << "[" << format_local_timestamp() << "] "
              << "[" << kLoggerName << "]: "
              << message
              << std::endl;
}

void print_help() {
    std::cout
        << "Usage: open_raw_radar_firmware_download\n"
        << "Download the radar firmware/meta image to the connected hardware once.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            if (arg == "--help" || arg == "-h") {
                print_help();
                return 0;
            }
            throw std::runtime_error("Unknown argument: " + arg);
        }

        log_line("INFO", "starting firmware download");
        const int result = AWR2243_firmwareDownload();
        if (result != 0) {
            std::ostringstream stream;
            stream << "firmware download failed with code " << result;
            log_line("ERROR", stream.str());
            return result;
        }

        log_line("INFO", "firmware download completed successfully");
        return 0;
    } catch (const std::exception& error) {
        log_line("ERROR", error.what());
        return 1;
    }
}
