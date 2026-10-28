#!/usr/bin/env bash
set -euo pipefail
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mode=${1:-inspect}
case "$mode" in
    inspect)
        exec python3 "$script_dir/inspect_linux_host.py"
        ;;
    install-base|enable-ssh)
        if [[ ! -r /etc/os-release ]]; then
            echo "Run this inside the booted Debian installation." >&2
            exit 2
        fi
        . /etc/os-release
        if [[ ${ID:-} != debian || ! ${VERSION_ID:-} =~ ^(12|13)(\.|$) ]]; then
            echo "Inspect this OS/release before installing packages; this helper targets Debian 12/13." >&2
            exit 2
        fi
        admin=()
        if [[ $EUID -ne 0 ]]; then admin=(sudo); fi
        "${admin[@]}" apt-get update
        if [[ $mode == install-base ]]; then
            "${admin[@]}" apt-get install -y ca-certificates git build-essential cmake ninja-build pkg-config python3 python3-pip python3-numpy python3-serial openssh-client v4l-utils usbutils pciutils iproute2
            echo "Base tools installed. ROS, venvs, firmware and SSH service were not configured."
        else
            "${admin[@]}" apt-get install -y openssh-server
            "${admin[@]}" systemctl enable --now ssh
            echo "SSH enabled using existing system account/authentication policy. Verify host keys and LAN access."
        fi
        ;;
    *)
        echo "Usage: bash tools/setup/debian_host.sh [inspect|install-base|enable-ssh]" >&2
        exit 2
        ;;
esac
