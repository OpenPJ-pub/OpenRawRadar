#pragma once

#include <filesystem>

namespace open_raw_radar::cli {

void prepare_output_owner(const std::filesystem::path& output_dir);
void finalize_output_owner(const std::filesystem::path& output_dir);

}  // namespace open_raw_radar::cli
