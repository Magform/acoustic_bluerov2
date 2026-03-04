# Acoustic Control for BlueROV2

Remote teleoperation of a BlueROV2 over an underwater acoustic link using ROS 2, DESERT Underwater, and the SuM software-defined modem.

## Overview

This repository provides the complete software stack for piloting a BlueROV2 over an acoustic communication link.

At the surface:

* Commands are generated from an Xbox controller (or a simulated joystick).
* They are transmitted acoustically using the SuM modem and the DESERT Underwater framework.
* Communication is handled through the `rmw_desert` ROS 2 middleware implementation.

Onboard the ROV:

* Commands are received and decoded.
* They are forwarded to the flight controller via MAVROS/MAVLink.
* The vehicle executes them in real time.

The system enables long-range underwater teleoperation where traditional RF communication is not viable.

---

# ROV Setup

These steps must be performed on a Raspberry Pi connected via LAN to the ROV’s onboard companion computer.

---

## 1. Flash and Install OS

Flash the SD card with **Ubuntu Server 24.04**.

---

## 2. Install Dependencies

### Install the SuM modem (if the Raspberry Pi is also acting as the modem)

If the Raspberry Pi is not the modem itself, ensure that a SuM modem is present on the same network.

Follow the official SubSeaPulse documentation:

[https://modem.subseapulse.com/sum-doc/](https://modem.subseapulse.com/sum-doc/)

---

### Install ROS 2

Follow the official installation guide for your ROS 2 distribution (e.g., Kilted):

```bash
# Official guide:
# https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html#install-ros-2
```

---

### Install `rmw_desert`

Follow the binary package instructions:

[https://github.com/signetlabdei/rmw_desert/wiki/Binary-packages](https://github.com/signetlabdei/rmw_desert/wiki/Binary-packages)

---

### Install MAVROS and geographic datasets

```bash
sudo apt install ros-kilted-mavros

wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

---

## 3. Configure Network

Configure a static IP address so the ROV companion computer is reachable at:

```
192.168.2.1
```

Create or edit:

```
/etc/netplan/01-rov.yaml
```

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.2.1/24
```

Apply the configuration:

```bash
sudo netplan apply
```

---

## 4. Test MAVROS

Before running the full acoustic stack, verify that MAVROS correctly communicates with the BlueROV2 flight controller.

Reference:
[https://discuss.bluerobotics.com/t/control-bluerov2-from-mavros/22451](https://discuss.bluerobotics.com/t/control-bluerov2-from-mavros/22451)

### Terminal 1 — Launch MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@
```

### Terminal 2 — Verify RC output

```bash
ros2 topic echo /mavros/rc/out
```

If data is being published, MAVROS is correctly connected.

---

### Useful Utility Commands

Arm the vehicle:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

Disarm the vehicle:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

Check autopilot state:

```bash
ros2 topic echo /mavros/state
```

Send manual RC override (example: slight forward thrust on channel 1):

```bash
ros2 topic pub /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
"channels: [1600, 1500, 1500, 1500, 1500, 1500, 1500, 1500]"
```

---

## 5. Run the ROV Stack

Open **six terminals** on the companion computer and execute the following commands **in order**:

| Terminal | Command                                                       | Description                                                                                              |
| -------- | ------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| T1       | `ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@` | MAVROS ↔ autopilot bridge                                                                                |
| T2       | `socat TCP-LISTEN:4000,fork TCP:127.0.0.1:55555`              | Forward modem TCP stream to DESERT or to the modem (change the IP if the RPi is not acting as the modem) |
| T3       | `./startmodem.sh`                                             | Start the SuM acoustic modem                                                                             |
| T4       | `python3 sub.py`                                              | Receive acoustic packets                                                                                 |
| T5       | `python3 pub.py 2>&1 \| tee received_log.log`                 | Decode and forward commands to MAVROS (with logging)                                                     |
| T6       | `ros2 topic echo /mavros/rc/override`                         | Monitor RC overrides (optional, for debugging)                                                           |

### Important

* **T3 must be executed inside:**

```
./sum/
```

* **T4 and T5 must be executed inside:**

```
/root/ros2_ws/src/acoustic_bluerov2/bridge
```

---

# Surface Station Setup

These steps are performed on the operator’s laptop at the surface.

---

## 1. Build and Enter Docker

```bash
cd docker/
sudo docker compose up -d
sudo docker exec -it acoustic-bluerov bash
```

Inside the container:

```bash
colcon build
source install/setup.bash
```

---

## 2. Run the Surface Stack

### Terminal 1 — Start the TCP tunnel toward the surface modem

```bash
# Replace 192.168.100.26 with the IP address of your surface modem
socat TCP-LISTEN:5000,fork TCP:192.168.100.26:55555
```

---

### Terminal 2 — Launch the controller node

#### With a physical Xbox controller connected:

```bash
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 run bluerov_controller xbox
```

---

#### Without a physical controller (software simulation)

You need **two terminals**.

Terminal A:

```bash
python3 /bridge/simulate_joystick.py
```

Terminal B:

```bash
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 run bluerov_controller xbox
```

---

# Notes

* The ROV bridge logs (`received_log.log`) are written in the working directory of Terminal T5.
  These CSV logs are used for post-deployment latency and packet delivery ratio (PDR) analysis.

* In the surface controller YAML configuration, setting:

```
threshold: -1
```

forces continuous retransmission of the last command.
This is recommended during real-world deployments to mitigate packet loss.

* Modulation schemes (e.g., JANUS vs. FlexFrame) are configured in:

  * `startmodem.sh`
  * DESERT configuration files in this repository