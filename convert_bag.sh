#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH=""
RADAR_CONFIG_FILENAME="AWR2243_mmwaveconfig_max15.txt"
DCA_CONFIG_FILENAME="dca_config.txt"
EXPORT_FORMAT="both"
OUTPUT_DIR=""
DEFAULT_BAG_ROOT="${ROOT_DIR}/output"

usage() {
  cat <<'EOF'
Usage:
  ./convert_bag.sh [options]

Options:
  --bag-path <dir>                 Path to rosbag2_* folder, default: latest under ./output
  --radar-config-filename <file>   Radar config file name under ./configs
  --dca-config-filename <file>     DCA config file name under ./configs
  --format <h5|bin|both>           Output format, default: both
  --output-dir <dir>               Output directory, default: <rosbag2_dir>/export
  -h, --help                       Show this help
EOF
}

find_latest_bag_path() {
  local latest_path
  latest_path="$(find "${DEFAULT_BAG_ROOT}" -mindepth 1 -maxdepth 1 -type d -name 'rosbag2_*' -print0 2>/dev/null \
    | xargs -0 ls -td 2>/dev/null \
    | head -n 1)"
  printf '%s' "${latest_path}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-path)
      BAG_PATH="$2"
      shift 2
      ;;
    --radar-config-filename)
      RADAR_CONFIG_FILENAME="$2"
      shift 2
      ;;
    --dca-config-filename)
      DCA_CONFIG_FILENAME="$2"
      shift 2
      ;;
    --format)
      EXPORT_FORMAT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  BAG_PATH="$(find_latest_bag_path)"
fi

if [[ -z "${BAG_PATH}" ]]; then
  echo "No rosbag2_* folder found under ${DEFAULT_BAG_ROOT}" >&2
  exit 1
fi

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "Bag path does not exist: ${BAG_PATH}" >&2
  exit 1
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${BAG_PATH}/export"
fi

RADAR_CONFIG_PATH="${ROOT_DIR}/configs/${RADAR_CONFIG_FILENAME}"
DCA_CONFIG_PATH="${ROOT_DIR}/configs/${DCA_CONFIG_FILENAME}"

python3 "${ROOT_DIR}/parser_tool/bag_convert.py" \
  --bag-path "${BAG_PATH}" \
  --radar-config-path "${RADAR_CONFIG_PATH}" \
  --dca-config-path "${DCA_CONFIG_PATH}" \
  --output-dir "${OUTPUT_DIR}" \
  --format "${EXPORT_FORMAT}"

echo "Post-process completed: ${OUTPUT_DIR}"
