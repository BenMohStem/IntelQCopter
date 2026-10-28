# Native Debian instead of WSL - 2026-10-28

The user confirmed that Debian is on a second drive in this same Windows PC. Booting Debian gives a real native Linux kernel and ordinary Linux tools; WSL is not required. The two native operating systems are not simultaneously running. Windows cannot SSH to the stopped Debian installation on its other drive. For simultaneous use, use a separate Linux computer or a deliberately configured VM; do not attach the live dual-boot disk to a VM as an automatic shortcut.

Windows read-only enumeration found a 256 GB Samsung GPT drive with an EFI partition and two unidentified partitions, alongside the 512 GB Windows C: drive. This is a candidate based on the user's description, not a filesystem/installation verification. No partition was mounted, formatted, offlined or changed. Do not use guessed partition numbers in mount or bootloader commands.

## First native session

1. Save work and manually restart through the existing firmware boot menu into the known Debian/GRUB entry. Do not reinstall, repartition or change bootloader settings from this guide. If that entry does not boot, stop and inspect the error instead.
2. In Debian, inspect release/architecture/storage without changing them:

```bash
cat /etc/os-release
uname -m
free -h
lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINTS
```

3. Use an existing checkout, or install the normal system prerequisites and clone the current project branch into a Linux-native source directory:

```bash
sudo apt update
sudo apt install git python3 ca-certificates
mkdir -p ~/src
git clone --depth 1 --single-branch --branch codex/motor-limits-system-tools https://github.com/BenMohStem/IntelQCopter.git ~/src/IntelQCopter
cd ~/src/IntelQCopter
bash tools/setup/debian_host.sh inspect
```

The inventory is read-only and does not capture images or start a server. Save output to `.local/` if needed; do not commit private device/network inventories. Windows Python venvs, ROS DLLs and build outputs cannot be reused as Linux binaries.

4. After reviewing Debian version, explicitly install common system tools:

```bash
bash tools/setup/debian_host.sh install-base
```

This installs system development, Python and device-inventory tools. It does not add a ROS repository, create a venv, flash an FC or start motor commands.

5. To administer this Debian host from the N150 or another simultaneously running computer on the LAN, explicitly enable SSH:

```bash
bash tools/setup/debian_host.sh enable-ssh
hostname -I
```

From that other machine, use your real username and measured address: `ssh USER@ADDRESS`. Verify the host key and configure normal SSH keys/account policy. Do not expose SSH directly to the Internet for this setup. When Windows is running on this same PC instead, Debian's SSH server is stopped because Debian itself is not running. [Debian SSH reference](https://www.debian.org/doc/manuals/debian-reference/ch06.en.html#_the_remote_access_server_and_utilities_ssh).

## ROS 2 selection after inventory

- Keep this project's Jazzy interfaces as the compatibility target initially.
- Ubuntu 24.04 amd64/arm64 is Jazzy Tier 1; Debian 12 amd64 is Tier 3. Debian 13 must be assessed separately. Do not add Ubuntu Noble apt sources to Debian or assume `apt install ros-jazzy-*` works natively. [REP-2000](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst).
- On the stronger Debian PC, evaluate a supported Ubuntu 24.04 ROS container or the documented source-build path matching the actual release. Neither is installed by the helper. Keep USB/camera permissions and DDS discovery explicit.
- On the N150, start with native Linux + SSH and camera/telemetry tests; add only the small compatible ROS pieces justified by measured memory/latency. Keep RViz/Gazebo and heavy builds on the stronger host.
- Keep source under `~/src`; keep per-OS build outputs separate. Do not share one ROS build/install tree across Windows/Linux. Preserve the existing Windows custom ROS/RViz/Gazebo sources and assets.

Pending: the first native Debian inventory, exact N150 model/CPU/RAM/camera/Wi-Fi identification, ROS installation strategy and physical FC serial adapters. No reboot or Linux session was performed remotely during this preparation.
