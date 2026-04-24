# BillyBot — Problems & Solutions Log

This document records every problem that was identified during development and the
solution that was implemented.  It exists so that a future session can pick up
without re-explaining the full history.

---

## Hardware Overview

| Component | Details |
|---|---|
| Main computer | Raspberry Pi 4 |
| Microcontroller | Raspberry Pi Pico 2 W (RP2350 + CYW43 Wi-Fi chip) |
| LiDAR | YDLidar X4 — `/dev/ydlidar` |
| Motor driver | WS55-220 BLDC, hall-sensor trapezoidal commutation (NOT FOC/ODrive) |
| Wheel spec | 190 mm diameter, 15:1 gear ratio |
| GPS | NEO-6M on Pico UART1 (GPIO 4/5), 9600 baud |
| Ultrasonic | 12 × HC-SR04 on Arduino Mega → USB serial `/dev/ttyACM1` |
| Linear actuator | Controlled via `actuator_cmd` Int32 topic (1=extend, -1=retract, 0=stop) |

---

## Problem 1 — Pico Would Not Connect on Boot (Required Physical Replug)

**Symptom:** After powering on the robot, the Pico (which drives the motors via
micro-ROS) would not communicate with the Pi.  The only fix was to physically
unplug and re-plug the USB cable between the Pico and the Pi.

**Root cause:** The original Pico firmware had no reconnection logic.  When the
micro-ROS agent on the Pi started, the Pico was already past its one-time init
and was not retrying.  If the agent was not ready at the exact moment the Pico
booted, communication never established.

**Solution (`pico_micro_ros_example.c`):**
A full agent state machine was added to the Pico firmware main loop:

```
WAITING_AGENT → (ping succeeds) → AGENT_AVAILABLE
              → (entities created) → AGENT_CONNECTED
              → (ping fails) → AGENT_DISCONNECTED
              → (cleanup done) → WAITING_AGENT  ← loops forever
```

The Pico now continuously pings the agent.  If the Pi reboots, the agent
restarts, or the USB enumerates late, the Pico automatically reconnects without
any physical intervention.  The onboard LED blinks slowly while searching and
goes solid when connected.

---

## Problem 2 — GPS Not Integrated Into the System

**Symptom:** The NEO-6M GPS module was wired to the Pico but nothing was reading
it or publishing it to ROS.  The web controller had no GPS display.

**Solution — Pico firmware (`pico_micro_ros_example.c`):**
- UART1 initialised on GPIO 4 (TX, unused) and GPIO 5 (RX from GPS) at 9600 baud.
- A lightweight NMEA parser reads `$GPRMC` / `$GNRMC` sentences from the GPS.
- Converts `DDMM.MMMMM` + hemisphere to signed decimal degrees.
- Publishes `sensor_msgs/NavSatFix` on the `/gps/fix` topic at 1 Hz.
- GPS polling runs regardless of ROS connection state (GPS buffers while
  reconnecting so no fix data is lost).

**Solution — Web controller (`robot_web_controller.py`):**
- Added a ROS 2 subscription to `/gps/fix`.
- Live latitude/longitude stored in a global and served via `/gps_data` Flask
  endpoint (JSON).
- Dashboard shows a "GPS" panel that polls every 2 seconds and displays
  coordinates or "No GPS fix".

**Solution — EKF fusion (`billybot_bringup/config/ekf_config.yaml`):**
- `robot_localization` EKF node fuses `/odom` (open-loop) and `/odometry/gps`
  (GPS converted to odom frame by `navsat_transform_node`).
- Provides a filtered odometry estimate that corrects open-loop drift over time.

---

## Problem 3 — LiDAR Not Working / Zero Points

**Symptom:** The YDLidar X4 would not publish any scan points.  SLAM Toolbox
received an empty or missing `/scan` topic.

**Root cause:** The SLAM config had incorrect frame names and scan topic settings,
and the launch file was not correctly passing parameters to SLAM Toolbox.

**Solution (`billybot_bringup/config/slam_config.yaml` and
`billybot_bringup/launch/slam.launch.py`):**
- Set `scan_topic: /scan`, `base_frame: base_link`, `odom_frame: odom`.
- Added a static TF publisher (`base_link → laser_frame`) so SLAM Toolbox can
  locate the LiDAR in the robot's coordinate frame.
- Tuned scan matching parameters for open-loop odometry (no encoders yet):
  - `minimum_travel_distance: 0.1` and `minimum_travel_heading: 0.1`
  - `scan_buffer_size: 10`, `do_loop_closing: true`
- `max_laser_range: 12.0` matches the X4's rated range.

**Note:** A `X4.yaml` LiDAR driver config file must exist at
`billybot_bringup/config/X4.yaml` on the Pi.  This is typically provided by the
`ydlidar_ros2_driver` package install.

---

## Problem 4 — No Ultrasonic Sensor Integration

**Symptom:** The 12 × HC-SR04 sensors on the Arduino Mega had no ROS integration.
Obstacle data was not available to the system, and there was no safety stop.

**Solution — Arduino firmware (`arduino_ultrasonic/arduino_ultrasonic.ino`):**
- Reads all 12 sensors sequentially (5 ms gap between each to prevent acoustic
  crosstalk).
- Sends a single packet per cycle over USB serial at 115200 baud:
  `US:d0:d1:d2:...:d11\r\n` (distances in metres, 3 decimal places).
- Out-of-range reads (no echo) return `4.50`.
- Pin assignments use even/odd pairs on Mega digital pins 22–45.

**Sensor layout (12 sensors, 3 rows × 4 columns):**
```
HIGH:  high_left  high_center_left  high_center_right  high_right   (pins 38-45)
MID:   mid_left   mid_center_left   mid_center_right   mid_right    (pins 30-37)
LOW:   low_left   low_center_left   low_center_right   low_right    (pins 22-29)
```

**Solution — ROS bridge (`billybot_bringup/billybot_bringup/ultrasonic_bridge.py`):**
- Reads the Arduino serial stream in a background thread.
- Publishes each sensor as `sensor_msgs/Range` on `/ultrasonic/<name>`.
- Serial port and baud are ROS parameters (default `/dev/ttyACM1`, 115200).
- If the Arduino is not connected, the node logs an error but does not crash.

**Solution — Safety node (`billybot_bringup/billybot_bringup/safety_node.py`):**
- Sits in the command pipeline: `/cmd_vel_raw` → safety check → `/cmd_vel`.
- Web controller publishes to `/cmd_vel_raw`; safety node is the sole publisher
  on `/cmd_vel` (what the motor controller actually reads).
- If any front sensor reads below 0.50 m, forward motion is replaced with a
  zero-velocity command.
- Backing up is always allowed so the robot can self-recover.
- Hysteresis: forward motion resumes only when all sensors clear 0.70 m
  (prevents rapid stop/start cycling at the threshold).
- **Fail-open:** if the Arduino/bridge is not running, commands pass straight
  through — robot still drives normally without ultrasonic hardware.

---

## Problem 5 — No Odometry (Encoders Not Yet Available)

**Symptom:** SLAM Toolbox requires an `odom → base_link` TF and a `/odom` topic,
but the robot has no wheel encoders yet (waiting on the differential receiver
hardware — see Pending Hardware below).

**Solution (`billybot_bringup/billybot_bringup/open_loop_odom.py`):**
- Integrates `/cmd_vel` commands over time using dead-reckoning math.
- Publishes `nav_msgs/Odometry` on `/odom` and broadcasts the `odom → base_link`
  TF at 20 Hz.
- This is a placeholder — it accumulates drift but gives SLAM Toolbox something
  to work with.  The GPS EKF partially corrects this drift.
- Once real encoders are available, replace this node with a hall-sensor-based
  publisher from the Pico firmware.

---

## ROS 2 Topic Architecture

```
[Web Browser]
     │  HTTP
     ▼
robot_web_controller.py  ──→  /cmd_vel_raw  ──→  safety_node  ──→  /cmd_vel  ──→  [Motors/Pico]
                                                      ▲
                              /ultrasonic/<name>  ────┘
                              (ultrasonic_bridge ← Arduino Mega)

[Pico 2 W]  ──→  /gps/fix  ──→  navsat_transform_node  ──→  /odometry/gps  ─┐
                                                                               ├──→  ekf_filter_node  ──→  /odometry/filtered
open_loop_odom  ──→  /odom  ──────────────────────────────────────────────────┘

ydlidar_ros2_driver  ──→  /scan  ──→  slam_toolbox  ──→  /map
```

---

## Connectivity

| Item | Value |
|---|---|
| Wi-Fi SSID | `BillyBot` |
| Wi-Fi Password | `BillyBot2024!` |
| Pi fixed IP | `10.42.0.1` |
| Web controller | `http://10.42.0.1:5000` |
| mDNS | `http://billybot.local:5000` |
| Web login | admin / BillyBot2024! (sha256 hashed in code) |

---

## Deployment Steps

### First time on a new Pi
```bash
chmod +x scripts/setup_billybot.sh
./scripts/setup_billybot.sh
```
This sets the hostname, installs avahi (mDNS), creates the Wi-Fi AP, installs
ROS dependencies, and registers the `micro_ros_agent` and `billybot_web`
systemd services to auto-start on boot.

### Build the ROS package
```bash
cd ~/your_ws
colcon build --packages-select billybot_bringup
source install/setup.bash
```

### Flash the Pico
Compile `pico_micro_ros_example.c` with the micro-ROS Pico SDK and copy the
`.uf2` to the Pico in BOOTSEL mode.

### Flash the Arduino Mega
Open `arduino_ultrasonic/arduino_ultrasonic.ino` in the Arduino IDE and upload
to the Mega.  Confirm `/dev/ttyACM1` is the correct port.

### Launch everything
```bash
ros2 launch billybot_bringup full_robot.launch.py
```
Optional override for the Arduino port:
```bash
ros2 launch billybot_bringup full_robot.launch.py arduino_port:=/dev/ttyACM0
```

The `micro_ros_agent` systemd service must be running before this launch.
It starts automatically on boot after `setup_billybot.sh` is run.

---

## Pending Hardware

**Differential receiver: AM26LV32EIDR (DigiKey) — ordered, not yet arrived.**

This IC converts the differential hall-sensor signals from the WS55-220 motor
driver into 3.3 V logic levels the Pico can read.

Once it arrives:
1. Wire the hall sensor A/B/C outputs from each motor through the AM26LV32EIDR
   to Pico GPIO pins.
2. Measure the motor pole pairs (or check the WS55-220 documentation).
3. Uncomment and fill in `MOTOR_POLE_PAIRS` in `pico_micro_ros_example.c`.
4. Add a hall-counter interrupt handler and `nav_msgs/Odometry` publisher to the
   Pico firmware.
5. Remove `open_loop_odom.py` from the launch file — replace with real encoder
   odometry.
6. Update `ekf_config.yaml` odom0 covariance values (open-loop values are
   currently set high to account for drift).

---

## Key Files Reference

| File | Purpose |
|---|---|
| `pico_micro_ros_example.c` | Pico firmware — reconnect loop, GPS parser, NavSatFix publisher |
| `robot_web_controller.py` | Flask + ROS 2 node — web UI, login, GPS display, cmd_vel publisher |
| `arduino_ultrasonic/arduino_ultrasonic.ino` | Arduino Mega firmware — HC-SR04 × 12, serial output |
| `billybot_bringup/billybot_bringup/safety_node.py` | Obstacle stop node |
| `billybot_bringup/billybot_bringup/ultrasonic_bridge.py` | Arduino serial → ROS Range topics |
| `billybot_bringup/billybot_bringup/open_loop_odom.py` | Dead-reckoning odometry (placeholder) |
| `billybot_bringup/launch/full_robot.launch.py` | Master launch — starts everything |
| `billybot_bringup/launch/slam.launch.py` | SLAM-only launch |
| `billybot_bringup/config/slam_config.yaml` | SLAM Toolbox tuning |
| `billybot_bringup/config/ekf_config.yaml` | robot_localization EKF + navsat config |
| `scripts/setup_billybot.sh` | One-time Pi setup (AP, services, deps) |

---

## Missing Information / Open Questions

- **Motor pole pairs** — unknown until AM26LV32EIDR arrives and can be measured.
- **GPS antenna position** — `full_robot.launch.py` has a static TF for
  `base_link → gps_link` at `[0.0, 0.0, 0.3]`.  Adjust the x/y/z offset to
  match where the NEO-6M antenna actually sits on the robot frame.
- **LiDAR position** — static TF currently set to `[0.1, 0.0, 0.2]` from
  `base_link`.  Adjust if the LiDAR is mounted differently.
- **`X4.yaml`** — present at `billybot_bringup/config/X4.yaml`. Sets
  `frame_id: laser_frame`, port `/dev/ydlidar`, `range_max: 12.0`. No action needed.
