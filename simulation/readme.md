# Simulation Guide

This guide explains how to configure and start the simulation environment for the BlueROV2 using Docker, DESERT, and ROS2.

---

## Docker Setup

1. Allow Docker to access the X server:
   ```bash
   xhost +local:docker
   ```

2. Start the Docker containers:
   ```bash
   docker compose up -d
   ```

3. Attach to the simulator container:
   ```bash
   docker exec -it simulator_desert-bluerov
   ```

---

## Starting the Simulation

> You'll need **5 terminal windows** open and connected to the Docker container.

---

### 1. RMW_DESERT

In the first terminal:

- Navigate to the DESERT test directory:
- Start the DESERT simulation:
  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  ns uwApplicationTCP.tcl
  ```

---

### 2. Acoustic Bridge (2 Terminals)

In two separate terminals:

- Terminal 2: Run the publisher
  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  python3 pub.py
  ```

- Terminal 3: Run the subscriber
  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  python3 sub.py
  ```

---

### 3. World Simulation

In terminal 4:

- Launch the simulated environment:
  ```bash
  ros2 launch bluerov2_description world_launch.py
  ```

---

### 4. Manual Control

In terminal 5:

- Use the following command to send thruster commands to the BlueROV:
  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 topic pub /bluerov2/cmd_thruster1 std_msgs/msg/Float32 "{data: 10}"
  ```

---

### 5. How use the controller

In terminal 5:
```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
   RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 run bluerov2_controller keyboard_thruster_controller --ros-args -p keymap_file:=/root/ros2_ws/src/acoustic_bluerov2/controller/src/bluerov2_controller/keymap.conf 
```