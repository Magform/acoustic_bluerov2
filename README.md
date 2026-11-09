# BlueROV2 sim over real modems

Gazebo plays the ROV, the real acoustic modems carry the commands (single-topic version).

```
xbox --> :5000 --socat--> surface modem ~~~ water ~~~ ROV modem <--socat-- :4000 <-- sub.py --> pub.py --> Gazebo
```

## Start

```bash
xhost +local:docker
docker compose up -d --build
# Once per terminal:
docker exec -it bluerov-sim-fixed bash
```

## Run  

```bash
# 1 - Gazebo
ros2 launch bluerov2_description world_launch.py

# 2 - tunnel to the "ROV" modem
socat TCP-LISTEN:4000,fork TCP:<ROV_MODEM_IP>:55555

# 3 + 4 - bridge (in the bridge folder, sub.py waits for pub.py if needed)
python3 pub.py
python3 sub.py

# 5 - tunnel to the surface modem
socat TCP-LISTEN:5000,fork TCP:<SURFACE_MODEM_IP>:55555

# 6 - controller
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ros2 run bluerov_controller xbox
```

Start the modems first (`./startmodem.sh`), and the socats before sub.py and the controller, otherwise rmw_desert has nothing to connect to.

Both bridge scripts log every command and pose they send or receive, with timestamps.

No modems? Skip the socats and sub/pub, run `python3 relay.py` instead and start the controller without `RMW_IMPLEMENTATION`/`DESERT_PORT`.

## Settings

Everything is in `bridge/config.yaml`. The ones that matter for the acoustic link:


## No Xbox pad? Use the keyboard

In one more terminal (bridge folder), before starting the controller:

```bash
python3 keyboard_joystick.py
```

It creates a fake joystick, so the controller can't tell the difference. Each key press is one step (25%):
W/S forward/back, A/D strafe, arrows up/down and yaw, SPACE stops everything, Q quits.
