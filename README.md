# Configuration

1) Flash SD card with Ubuntu Server 24.04

2) Install sum modem things

3) Install ROS2 [https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html#install-ros-2]

4) Install mavro [sudo apt install ros-kilted-mavros + wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh]

5) Set lan ip to 192.168.2.1
/etc/netplan/01-rov.yaml

network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.2.1/24]

sudo netplan apply

6) Test mavros:
Termianl1: ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@
Terminal2: ros2 topic echo /mavros/rc/out

7)
