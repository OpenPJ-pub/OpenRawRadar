#include "platform/output_owner.h"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <sys/types.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace open_raw_radar::cli {
namespace {

struct OwnerMapping {
    uid_t uid{0};
    gid_t gid{0};
    bool valid{false};
};

OwnerMapping resolve_owner_mapping() {
    const char* sudo_uid = std::getenv("SUDO_UID");
    const char* sudo_gid = std::getenv("SUDO_GID");
    if (sudo_uid == nullptr || sudo_gid == nullptr) {
        return {};
    }

    OwnerMapping mapping;
    mapping.uid = static_cast<uid_t>(std::stoul(sudo_uid));
    mapping.gid = static_cast<gid_t>(std::stoul(sudo_gid));
    mapping.valid = true;
    return mapping;
}

void apply_owner_if_needed(const fs::path& path, const OwnerMapping& owner_mapping) {
    if (!owner_mapping.valid) {
        return;
    }
    if (::chown(path.c_str(), owner_mapping.uid, owner_mapping.gid) != 0) {
        throw std::runtime_error("Failed to apply output ownership: " + path.string());
    }
}

}  // namespace

void prepare_output_owner(const std::filesystem::path& output_dir) {
    apply_owner_if_needed(output_dir, resolve_owner_mapping());
}

void finalize_output_owner(const std::filesystem::path& output_dir) {
    const auto owner_mapping = resolve_owner_mapping();
    if (!owner_mapping.valid || !fs::exists(output_dir)) {
        return;
    }

    for (const auto& entry : fs::recursive_directory_iterator(output_dir)) {
        apply_owner_if_needed(entry.path(), owner_mapping);
    }
    apply_owner_if_needed(output_dir, owner_mapping);
}

}  // namespace open_raw_radar::cli
