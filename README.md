<div align="justify">

# Lenna Mobile Robot ONE

| <img src="./docs/figures/lenna-amr-one.jpg" alt="Lenna Mobile Robot ONE v.1.0.0" width="585"/> | <img src="./docs/figures/lenna-amr-one-v2.jpg" alt="djkstra" width="512"/> |
|:--:|:--:| 
| Lenna Mobile Robot ONE | LMRO Revised Model |

## Introduction
Lenna Autonomous Mobile Robot is designed for educational and research purposes. It is built upon a robust hardware architecture that combines an ARM-based embedded microcontroller and a Jetson Nano single-board computer (SBC) to provide both low-level and high-level control capabilities.

At the core of the robot is the ARM-based embedded microcontroller, which is responsible for the real-time control of the motors and the reading of various sensors. This includes implementing motor control using PID (Proportional-Integral-Derivative) algorithms and efficiently handling sensor data, such as from inertial measurement units and wheel encoders. The embedded microcontroller enables the robot to perform precise dead-reckoning navigation by fusing odometry information from the encoders.

The Jetson Nano SBC, on the other hand, is dedicated to executing higher-level tasks. It is responsible for processing data from sensors like cameras and LiDAR units, enabling the robot to perceive its environment. This allows the implementation of advanced navigation algorithms, such as Simultaneous Localization and Mapping (SLAM), which combine odometry and laser scan data to build a map of the surroundings and localize the robot within it.

## Folder Structure

    .
    ├── hardware/                # Design & implementation
    │   ├── mechanics/           # CAD, 3D models, assembly instructions
    │   ├── electronics/         # Schematics, PCB layouts, wiring
    │   └── bill_of_materials/   # Parts list
    │
    ├── software/
    │   ├── embedded/            # Microcontroller code (C/C++/RTOS)
    │   ├── ros/                 # ROS packages, launch files, nodes
    │   └── simulation/          # Gazebo, test worlds, HIL
    │
    ├── docs/
    │   ├── user_manual/         # Setup, usage, safety
    │   ├── developer_guide/     # Contribution, architecture, coding style
    │   └── figures/             # Images, diagrams, plots
    │
    ├── examples/                # Sample scripts, demos
    └── README.md                # Entry point, quick start

</div>