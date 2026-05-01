#include <exception>
#include <iostream>

#include "capture_app.h"

int main(int argc, char* argv[]) {
    try {
        return open_raw_radar::cli::run_capture_app(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << std::endl;
        return 1;
    }
}
