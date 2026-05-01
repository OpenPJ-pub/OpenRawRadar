#!/bin/bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  sudo bash ./macvlan_config.sh <parent_iface> <subnet>

Environment fallback:
  RADAR_PARENT_IFACE
  SUBNET

Examples:
  sudo bash ./macvlan_config.sh enp5s0 33
  sudo -E bash ./macvlan_config.sh
EOF
}

require_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        echo "This script must be run as root." >&2
        exit 1
    fi
}

main() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        exit 0
    fi

    require_root

    local parent_iface="${1:-${RADAR_PARENT_IFACE:-}}"
    local subnet="${2:-${SUBNET:-33}}"
    if [[ -z "${parent_iface}" ]]; then
        echo "Missing parent interface." >&2
        usage
        exit 1
    fi

    local shim_ip="192.168.${subnet}.1/24"

    echo ">>> parent iface: ${parent_iface}"
    echo ">>> subnet: ${subnet}"
    echo ">>> shim ip: ${shim_ip}"

    ip link del macvlan-shim 2>/dev/null || true
    ip link add macvlan-shim link "${parent_iface}" type macvlan mode bridge
    ip addr add "${shim_ip}" dev macvlan-shim
    ip link set macvlan-shim up
}

main "$@"
