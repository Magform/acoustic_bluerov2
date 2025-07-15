# Acoustic BlueROV2

## Objective

The goal of this project is to control a **BlueROV2** via **acoustic communication** and an **XBOX controller**, first within a simulation environment and eventually in the real world.
The simulation uses **Gazebo** to visualize the BlueROV2 and **DESERT** to emulate the underwater acoustic communication channel.

---

## How to Use

> Ensure Docker is installed and properly configured to support **ROS2** and **Gazebo**.

### 1. Docker Setup

1. Allow Docker access to the X server (for Gazebo visualization):

   ```bash
   xhost +local:docker
   ```

2. Start the Docker containers:

   ```bash
   cd ./simulation/
   docker compose up -d
   ```

3. Attach to the simulation container:

   ```bash
   docker exec -it simulator_desert-bluerov bash
   ```

---

### 2. Starting the Simulation

> You will need **five terminals**, all attached to the Docker container.

#### Terminal 1 – RMW\_DESERT

Start the DESERT acoustic simulation:

```bash
cd /root/ros2_ws/src/acoustic_bluerov2/bridge
ns uwApplicationTCP.tcl
```

#### Terminals 2 & 3 – Acoustic Bridge

These two scripts handle data translation between Gazebo and the acoustic middleware (`rmw_desert`).

* **Terminal 2 – Publisher**
  Reads commands from the subscriber and publishes them to Gazebo using ROS2:

  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  python3 pub.py
  ```

* **Terminal 3 – Subscriber**
  Receives acoustic data from the `rmw_desert` middleware and forwards them to the publisher:

  ```bash
  cd /root/ros2_ws/src/acoustic_bluerov2/bridge
  python3 sub.py
  ```

#### Terminal 4 – World Simulation

Launch the Gazebo simulation world:

```bash
ros2 launch bluerov2_description world_launch.py
```

#### Terminal 5 (Option 1) – Manual Thruster Command

Send a specific force (in Newtons) to a thruster (e.g., 10N to thruster 1):

```bash
cd /root/ros2_ws/src/acoustic_bluerov2/bridge
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 topic pub /bluerov2/cmd_thruster1 std_msgs/msg/Float32 "{data: 10}"
```

#### Terminal 5 (Option 2) – Xbox Controller

Use the Xbox controller to control the BlueROV2:

```bash
cd /root/ros2_ws/src/acoustic_bluerov2/bridge
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 run bluerov_controller xbox
```

#### Terminal 6 – Read Position

To read the position of the BlueROV2:

```bash
ros2 topic echo /bluerov2/pose_gt geometry_msgs/msg/Pose
```

---

## Controller Configuration

The controller reads configuration from a file in the current working directory (`/root/ros2_ws/src/acoustic_bluerov2/bridge`).
This file defines control parameters and joystick mapping:

```yaml
max_speed: 10.0                 # Maximum speed (N)
sending_time: 1.0               # Time interval for sending messages (s)
thruster: 6                     # Number of thrusters
threshold: 0.1                  # Joystick threshold for sending a message
pose: /bluerov2/pose_gt         # Topic for sending back position (set NONE to disable)
keymap:
  ax0: [-1, -1, -1, -1, 0, 0]   # Vertical Left Axis
  ax1: [-1, 1, 1, -1, 0, 0]     # Horizontal Left Axis
  ax2: [0, 0, 0, 0, -1, 1]      # Vertical Right Axis
  ax3: [0, 1, -1, 0, 0, 0]      # Horizontal Right Axis
  ax4: [0, 0, 0, 0, 0, 0]       # Left Trigger
  ax5: [0, 0, 0, 0, 0, 0]       # Right Trigger
```

> You can adapt this mapping to your controller model or preferred control behavior.

---

## Conclusions & Future Developments

* Integrate with real BlueROV2 hardware using actual acoustic modems.
* Optimize the latency and robustness of the acoustic link.
* Enable full 6-DOF manual control through the Xbox controller.
* Develop autonomous mission capabilities (e.g., waypoint navigation) over the acoustic channel.