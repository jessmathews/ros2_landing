# ROS2 Autonomous Drone Control
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter%204.x-green)](https://ardupilot.org/)

 > ROS2 autonomous missions with mavros. Tested in SITL and hardware

Validated on real flights with Cube Orange flight controller and Raspberry Pi 4.

## Key Features

- **Companion Computer Control** - Command drone autonomously from Raspberry Pi in GUIDED mode
- **Ground Station Optional** - Mission Planner/QGroundControl for monitoring
- **Testing Scripts** - Testing Scripts for verification

## This repo contains
```
launch/
├── gazebo_stream.sh [ start GStreamer cam streaming from gazebo ]
├── mavros_real.sh [ Mavros for Hardware ]
└── mavros.sh [ Mavros for SITL ] 
scripts/
    ├── fullmiss.py
    ├── qr_test.py
    ├── test_mission.py
    └── test_scripts
        ├── arm.py
        └── takeoff.py
```
