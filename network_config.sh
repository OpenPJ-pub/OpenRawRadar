#!/bin/bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  sudo bash ./network_config.sh <parent_iface> [ring_size|auto] [mtu] [socket_buffer] [backlog]

Environment fallback:
  RADAR_PARENT_IFACE
  RADAR_RING_SIZE
  RADAR_MTU
  RADAR_SOCKET_BUFFER
  RADAR_NETDEV_BACKLOG

Examples:
  sudo bash ./network_config.sh enp5s0 auto 9000
  sudo -E bash ./network_config.sh
EOF
}

require_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        echo "This script must be run as root." >&2
        exit 1
    fi
}

resolve_ring_size() {
    local iface="$1"
    local requested="$2"
    if [[ "${requested}" != "auto" ]]; then
        echo "${requested}"
        return
    fi

    local max_rx
    local max_tx
    max_rx="$(ethtool -g "${iface}" 2>/dev/null | awk '/Pre-set maximums:/{flag=1; next} flag && $1=="RX:" {print $2; exit}')"
    max_tx="$(ethtool -g "${iface}" 2>/dev/null | awk '/Pre-set maximums:/{flag=1; next} flag && $1=="TX:" {print $2; exit}')"

    if [[ -z "${max_rx}" || -z "${max_tx}" ]]; then
        echo "Failed to read ring size limits for ${iface}." >&2
        exit 1
    fi

    if (( max_rx < max_tx )); then
        echo "${max_rx}"
    else
        echo "${max_tx}"
    fi
}

main() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        exit 0
    fi

    require_root

    local parent_iface="${1:-${RADAR_PARENT_IFACE:-}}"
    local ring_request="${2:-${RADAR_RING_SIZE:-auto}}"
    local mtu="${3:-${RADAR_MTU:-9000}}"
    local socket_buffer="${4:-${RADAR_SOCKET_BUFFER:-100000000}}"
    local backlog="${5:-${RADAR_NETDEV_BACKLOG:-6000}}"

    if [[ -z "${parent_iface}" ]]; then
        echo "Missing parent interface." >&2
        usage
        exit 1
    fi

    local ring_size
    ring_size="$(resolve_ring_size "${parent_iface}" "${ring_request}")"

    echo ">>> parent iface: ${parent_iface}"
    echo ">>> ring size: ${ring_size}"
    echo ">>> mtu: ${mtu}"
    echo ">>> socket buffer: ${socket_buffer}"
    echo ">>> backlog: ${backlog}"

    ethtool -G "${parent_iface}" tx "${ring_size}"
    ethtool -G "${parent_iface}" rx "${ring_size}"
    ethtool -g "${parent_iface}"

    sysctl -w net.core.wmem_default="${socket_buffer}"
    sysctl -w net.core.wmem_max="${socket_buffer}"
    sysctl -w net.core.rmem_default="${socket_buffer}"
    sysctl -w net.core.rmem_max="${socket_buffer}"
    sysctl -w net.core.netdev_max_backlog="${backlog}"

    ifconfig "${parent_iface}" mtu "${mtu}" up
}

main "$@"
