# ROS2 DWM1001 Package
This repo contains a ROS2 driver for the Qorvo DWM1001 Indoor Positioning System - ported from a forked version of the [ros-dwm1001-uwb-localization](https://github.com/TIERS/ros-dwm1001-uwb-localization) repository and with additional features added such as:
- tty port detection based on device serial numbers and device chip names
- math packages for usage of multiple devices for corrected heading.

This package was ported to ROS2 for use in the CARMA 1tenth scaled-down cooperative driving automation program. The code has been tested in Ubuntu 20.04/ROS2 Foxy.

## Installation
Clone this repo into your ROS2 workspace and build it with:
```
colcon build --symlink-install --packages-select dwm1001_ros2
```

Install python dependencies with
```
sudo pip install pyserial

```

## Configuration
This package assumes that the DWM1001-dev device is connected by USB.
Additionally, CARMA 1tenth vehicles have multiple devices connected over USB - and the tty port of the device may not always be the same.
A parameter file will need to be modified with the intended device's serial number and device description.
To find these values:
1. connect the dwm1001-dev device to your computer
2. run the following command in the terminal:
```
python -m serial.tools.list_ports -v
```
The device description will be the value following `desc:`.
The serial number will be the value following `SER=`.