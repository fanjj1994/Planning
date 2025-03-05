# Planning
## Project Structure

The project is organized as follows:

```
├── build
├── doc
│   └── UtilizeClangFormat.md
├── install
├── log
├── README.md
├── scripts
│   └── format_all.sh
└── src
    ├── base_msgs
    │   ├── CMakeLists.txt
    │   ├── msgs
    │   ├── package.xml
    │   └── srv
    ├── data_plot
    │   ├── data_plot
    │   │   ├── data_plot.py
    │   │   └── __init__.py
    │   ├── package.xml
    │   ├── resource
    │   │   └── data_plot
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── test
    │       ├── test_copyright.py
    │       ├── test_flake8.py
    │       └── test_pep257.py
    └── planning_core
        ├── CMakeLists.txt
        ├── config
        ├── launch
        ├── package.xml
        ├── rviz
        ├── src
        │   ├── common
        │   │   ├── CMakeLists.txt
        │   │   ├── common_type
        │   │   │   └── common_type.h
        │   │   ├── config_reader
        │   │   │   ├── config_reader.cpp
        │   │   │   └── config_reader.h
        │   │   └── math
        │   │       ├── curve.cpp
        │   │       ├── curve.h
        │   │       ├── polynomial_curve.cpp
        │   │       └── polynomial_curve.h
        │   ├── decision_center
        │   │   ├── CMakeLists.txt
        │   │   ├── decision_center.cpp
        │   │   └── decision_center.h
        │   ├── global_planner
        │   │   ├── CMakeLists.txt
        │   │   ├── global_path_server.cpp
        │   │   ├── global_path_server.h
        │   │   ├── global_planner_base.h
        │   │   └── global_planner_normal
        │   │       ├── global_planner_normal.cpp
        │   │       └── global_planner_normal.h
        │   ├── local_planner
        │   │   ├── CMakeLists.txt
        │   │   ├── local_path
        │   │   │   ├── local_path_planner.cpp
        │   │   │   ├── local_path_planner.h
        │   │   │   ├── local_path_smoother.cpp
        │   │   │   └── local_path_smoother.h
        │   │   ├── local_speeds
        │   │   │   ├── local_speeds_planner.cpp
        │   │   │   ├── local_speeds_planner.h
        │   │   │   ├── local_speeds_smoother.cpp
        │   │   │   └── local_speeds_smoother.h
        │   │   ├── local_trajectory_combiner.cpp
        │   │   └── local_trajectory_combiner.h
        │   ├── move_cmd
        │   │   ├── CMakeLists.txt
        │   │   ├── ego_car_move_cmd.cpp
        │   │   ├── ego_car_move_cmd.h
        │   │   ├── tp_move_cmd.cpp
        │   │   └── tp_move_cmd.h
        │   ├── planning_process
        │   │   ├── CMakeLists.txt
        │   │   ├── planning_node.cpp
        │   │   ├── planning_process.cpp
        │   │   └── planning_process.h
        │   ├── pnc_map_creator
        │   │   ├── CMakeLists.txt
        │   │   ├── pnc_map_creator_base.h
        │   │   ├── pnc_map_server.cpp
        │   │   ├── pnc_map_server.h
        │   │   ├── pnc_map_straight
        │   │   │   ├── pnc_map_creator_straight.cpp
        │   │   │   └── pnc_map_creator_straight.h
        │   │   └── pnc_map_sturn
        │   │       ├── pnc_map_creator_sturn.cpp
        │   │       └── pnc_map_creator_sturn.h
        │   ├── reference_line
        │   │   ├── CMakeLists.txt
        │   │   ├── reference_line_creator.cpp
        │   │   ├── reference_line_creator.h
        │   │   ├── reference_line_smoother.cpp
        │   │   └── reference_line_smoother.h
        │   └── vehicle_info
        │       ├── CMakeLists.txt
        │       ├── ego_car
        │       │   ├── ego_car_base.cpp
        │       │   └── ego_car_base.h
        │       ├── traffic_participants
        │       │   ├── tp_base.cpp
        │       │   └── tp_base.h
        │       └── vehicle_info_base.h
        └── urdf
            ├── ego_car_model
            └── tp_model
```

## Overview

This project is part of the Advanced Driver Assistance Systems (ADAS) and focuses on the planning module. The planning module is responsible for generating a safe and efficient path for the vehicle to follow. The project depends on ROS 2 Humble and is recommended to run on Ubuntu 22.04.

## Pre-Requistes

Before building the project, ensure you have the following dependencies installed from their GitHub repositories:

- Eigen 3.4.0
- osqp
- yaml-cpp
- matplotlib

You can install these dependencies using the following commands:


### Install Eigen 3.4.0
https://eigen.tuxfamily.org
recommended version: 3.4.0
```sh
sudo mv eigen-3.4.0/ /usr/local/include/
mkdir build
```


### Install osqp
```sh
git clone https://github.com/oxfordcontrol/osqp.git
cd osqp
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

### Install yaml-cpp
```sh
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

### Install matplotlib
```sh
pip install matplotlib
```

## Getting Started

To get started with the project, follow these steps:

1. Clone the repository:
    ```sh
    git clone https://github.com/fanjj1994/ADAS.git
    ```
2. Navigate to the planning directory:
    ```sh
    cd /home/fanjj1994/ADAS/planning
    ```
3. Build the project using ROS 2 Humble:
    ```sh
    source /opt/ros/humble/setup.bash
    mkdir -p build
    cd build
    colcon build
    ```

## Running Tests

TBD

## Contributing

Contributions are welcome! Please read the [contributing guidelines](docs/contributing.md) first.

## Reference
ROS2 Humble Documentation: https://docs.ros.org/en/humble/index.html