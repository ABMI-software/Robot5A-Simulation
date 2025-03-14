# Robotic Arm Simulation with ROS2 Humble

<p align="center">
  <img src="images/rviz_view.png" alt="Robotic Arm" />
</p>


## Table of Contents

- [Project Description](#project-description)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Simulation and controlling the robotic arm](#launching-the-simulation-and-controlling-the-robotic-arm)
- [Architecture](#architecture)
- [Technologies Used](#technologies-used)
- [Contributing](#contributing)
- [Documentation](#documentation)
- [License](#license)

## Project Description

This project simulates and controls a robotic arm using **ROS2 Humble**, **Gazebo**, and **MoveIt2**. The robotic arm is defined using URDF (Unified Robot Description Format) and is capable of executing planned trajectories within a simulated environment. The simulation integrates ROS2 control with MoveIt2 for advanced motion planning and trajectory execution. Additionally, the robotic arm is controlled using a camera and ArUco markers placed on the robot, workspace, and objects, enabling precise object pick and place operations through image processing and computer vision techniques.

## Features

- **Simulation Environment**: Realistic simulation using Gazebo.
- **Motion Planning**: Advanced trajectory planning with MoveIt2.
- **Control Mechanism**: Camera-based control using ArUco markers.
- **Object Manipulation**: Pick and place objects within the simulated workspace.
- **Camera Calibration**: Accurate camera setup for reliable image processing.
- **Modular Design**: Integration of Python and C++ for flexibility and performance.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS2 Distribution**: ROS2 Humble
- **Simulation Environment**: Gazebo
- **Motion Planning**: MoveIt2
- **Additional Tools**:
  - `meshlab` for mesh processing

## Installation

### Using Debian/Ubuntu

1. **Update Package Lists**

    ```bash
    sudo apt update
    ```

2. **Install ROS2 Humble and Required Packages**

    ```bash
    sudo apt-get install -y \
        libgeometric-shapes-dev \
        meshlab \
        ros-humble-diagnostic-updater \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-state-publisher \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit \
        ros-humble-robot-state-publisher \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ros2bag \
        ros-humble-rqt \
        ros-humble-rqt-graph \
        ros-humble-rviz2 \
        ros-humble-tf2-tools \
        ros-humble-xacro
    ```


3. **Clone the Repository**

    ```bash
    cd ~/
    git clone https://github.com/ABMI-software/Robot5A-Simulation.git
    ```

4. **Build the Workspace**

    ```bash
    cd Robot5A-Simulation
    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

    ```bash
    colcon build
    ```

5. **Source the Workspace**

    ```bash
    source install/setup.bash
    ```

6. **Export the settings**

    ```bash
    export GAZEBO_RESOURCE_PATH="~/Robot5A-Simulation/src/robot_description/:/usr/share/gazebo-11/models:/usr/share/gazebo-11"
    export QT_QPA_PLATFORM=xcb
    ```


7. **Write to bashrc for automatic export and source when launching the terminal**

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/Robot5A-Simulation/install/setup.bash" >> ~/.bashrc
    echo "export GAZEBO_RESOURCE_PATH="~/Robot5A-Simulation/src/robot_description/:/usr/share/gazebo-11/models:/usr/share/gazebo-11"" >> ~/.bashrc
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc
    ```

### Rebuild your Project

1. **Build Clean**

    ```bash
    cd ~/Robot5A-Simulation
    rm -rf build install log
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-clean-cache
    ```


### Using Docker and dosh

Alternatively, you may consider using [Docker](https://www.docker.com/) and the wrapper [dosh](https://gportay.github.io/dosh/).

1. **Install docker**

    Follow its [documentation](https://docs.docker.com/engine/install/debian/).

2. **Manage Docker as a non-root user**

    Follow its [documentation](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```

3. **Install dosh**

    Follow its [README](https://github.com/gportay/dosh?tab=readme-ov-file#install).

    ```bash
    git clone https://github.com/gportay/dosh.git
    cd dosh
    make user-install
    ```

4. **Run an interactive docker shell**

    ```bash
    dosh
    ```

5. **Enter any commands from the shell**

    The commands are run in the container, with the user credential, the current directory bind mounted, and few other things (see [doshrc](doshrc)).
    The image is based on the official docker image `ros:humble-ros-base` from [dockerhub](https://hub.docker.com/_/ros), it is currently derived from Ubuntu Jammy (22.04).

__Note__: The run-command file [.bashrc](.bashrc) sources the necessary bits so it is unecessary to source the files `/opt/ros/humble/setup.bash` and `install/setup.bash` manually.


## Usage

### Launching the Simulation and controlling the robotic arm

Start the Gazebo simulation environment with the robotic arm using MoveIt.

```bash
ros2 launch robot_control visual_sim.launch.py 
```
N.B. : It's also possible to control the arm using rviz2

## Architecture

The project is structured into several key components:

- **URDF Definition**: Defines the physical and visual properties of the robotic arm.
- **ROS2 Control**: Manages the hardware interfaces and controllers for joint movements.
- **MoveIt2 Integration**: Handles motion planning and trajectory execution.
- **Gazebo Simulation**: Provides a realistic environment for testing and development.
- **Camera and ArUco Markers**: Facilitates precise control and object tracking through computer vision.
- **Python and C++ Nodes**: Implements control logic, image processing, and communication between components.

## Technologies Used

- **ROS2 Humble**: Robot Operating System for middleware and communication.
- **Gazebo**: Simulation environment for robotics.
- **MoveIt2**: Motion planning framework.
- **URDF**: Robot description format.
- **Python**: Scripting and automation.
- **C++**: Performance-critical components.
- **OpenCV**: Image processing and computer vision.
- **ArUco**: Marker-based tracking.

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork the Repository**
2. **Create a Feature Branch**

    ```bash
    git checkout -b feature/YourFeature
    ```

3. **Commit Your Changes**

    ```bash
    git commit -m "Add your feature"
    ```

4. **Push to the Branch**

    ```bash
    git push origin feature/YourFeature
    ```

5. **Open a Pull Request**

## Documentation

All C++ nodes created under robot control package are documented using Doxygen check the **docs/html/index.html** for an interactive code docs

## License

This project is licensed under the MIT License.

---

*Developed by [Eliott, Omar & Matthieu - ABMI Groupe](https://github.com/ABMI-software/Robot5A-Simulation)*


