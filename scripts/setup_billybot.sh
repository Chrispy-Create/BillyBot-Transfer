#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# BillyBot one-time setup script
# Run once on the Raspberry Pi 4 as a user with sudo access.
#
# What this does:
#   1. Sets the hostname to "billybot"
#   2. Installs avahi-daemon (mDNS — enables billybot.local)
#   3. Creates a persistent WiFi access point ("BillyBot" network)
#      The Pi's built-in WiFi becomes the AP; use Ethernet for internet.
#   4. Installs required ROS 2 Python/apt packages
#   5. Installs systemd services for:
#        micro_ros_agent   — Pico ↔ ROS 2 bridge (USB serial)
#        billybot_web      — Flask web controller (auto-starts on boot)
#
# Usage:
#   chmod +x setup_billybot.sh && ./setup_billybot.sh
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SSID="BillyBot"
WIFI_PASSWORD="BillyBot2024!"
HOSTNAME="billybot"
AP_IP="10.42.0.1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo ""
echo "╔══════════════════════════════════════╗"
echo "║      BillyBot Setup Script           ║"
echo "╚══════════════════════════════════════╝"
echo ""

# ── 1. Hostname ───────────────────────────────────────────────────────────────
echo "[1/5] Setting hostname to '$HOSTNAME'..."
sudo hostnamectl set-hostname "$HOSTNAME"
sudo sed -i "s/127\.0\.1\.1.*/127.0.1.1\t$HOSTNAME/" /etc/hosts || \
    echo "127.0.1.1    $HOSTNAME" | sudo tee -a /etc/hosts > /dev/null

# ── 2. mDNS (billybot.local) ──────────────────────────────────────────────────
echo "[2/5] Installing avahi-daemon (mDNS)..."
sudo apt-get update -qq
sudo apt-get install -y avahi-daemon
sudo systemctl enable --now avahi-daemon

# ── 3. WiFi Access Point ──────────────────────────────────────────────────────
echo "[3/5] Configuring WiFi access point (SSID: $SSID)..."

# Remove old connection if it exists
sudo nmcli con delete billybot-ap 2>/dev/null || true

sudo nmcli con add \
    type wifi \
    ifname wlan0 \
    con-name billybot-ap \
    autoconnect yes \
    ssid "$SSID"

sudo nmcli con modify billybot-ap \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv4.addresses "$AP_IP/24"

sudo nmcli con modify billybot-ap \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$WIFI_PASSWORD"

sudo nmcli con up billybot-ap

echo "    AP active — SSID: $SSID | IP: $AP_IP"

# ── 4. ROS 2 / Python dependencies ───────────────────────────────────────────
echo "[4/5] Installing ROS 2 and Python dependencies..."
sudo apt-get install -y \
    python3-serial \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros

pip3 install flask --quiet 2>/dev/null || true

# ── 5. Systemd services ───────────────────────────────────────────────────────
echo "[5/5] Installing systemd services..."

# Detect the ROS 2 setup file location
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$HOME/ros2_ws/install/setup.bash"

# micro-ROS agent service
# Adjust the serial port if the Pico appears on a different /dev/ttyACM*
sudo tee /etc/systemd/system/micro_ros_agent.service > /dev/null <<EOF
[Unit]
Description=micro-ROS Agent (Pico 2 W bridge)
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=$USER
ExecStart=/bin/bash -c "source $ROS_SETUP && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# BillyBot web controller service
sudo tee /etc/systemd/system/billybot_web.service > /dev/null <<EOF
[Unit]
Description=BillyBot Web Controller
After=micro_ros_agent.service
StartLimitIntervalSec=0

[Service]
Type=simple
User=$USER
Environment="ROS_DOMAIN_ID=0"
ExecStartPre=/bin/sleep 3
ExecStart=/bin/bash -c "source $ROS_SETUP && python3 $REPO_DIR/robot_web_controller.py"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable micro_ros_agent.service
sudo systemctl enable billybot_web.service
sudo systemctl start  micro_ros_agent.service
sudo systemctl start  billybot_web.service

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Setup complete!                                         ║"
echo "║                                                          ║"
echo "║  WiFi:     $SSID                                  ║"
echo "║  Password: $WIFI_PASSWORD                          ║"
echo "║                                                          ║"
echo "║  Connect to BillyBot WiFi, then open:                   ║"
echo "║    http://$AP_IP:5000                              ║"
echo "║    http://billybot.local:5000  (mDNS)                   ║"
echo "║                                                          ║"
echo "║  Default login:  admin / BillyBot2024!                  ║"
echo "║  Change with:  BILLYBOT_ADMIN_PASSWORD env var          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
