<h1>ROS 2 Nav2 Foundations</h1>

---

**Contents**:

- [About ROS 2](#about-ros-2)
- [About Navigation2 (Nav2)](#about-navigation2-nav2)
- [ROS 2 Humble](#ros-2-humble)
- [ROS 2 installation (Humble)](#ros-2-installation-humble)
- [Nav2 stack installation and related installations](#nav2-stack-installation-and-related-installations)
  - [Preliminary points](#preliminary-points)
  - [Packages to install](#packages-to-install)
- [Other useful ROS 2 installations](#other-useful-ros-2-installations)
  - [Turtlebot3](#turtlebot3)
  - [URDF Tutorial](#urdf-tutorial)
  - [SLAM Toolbox](#slam-toolbox)
- [ROS 2 command line](#ros-2-command-line)
- [Division of navigation-related operations](#division-of-navigation-related-operations)
- [Further reading](#further-reading)
  - [Linux and ROS 2 related](#linux-and-ros-2-related)
  - [Nav2-related](#nav2-related)
  - [Procedure outlines](#procedure-outlines)
  - [Associated research](#associated-research)

---

> **Reference course**: [_ROS 2 Nav2 [Navigation 2 Stack] - with SLAM and Navigation_, **udemy.com**](https://www.udemy.com/course/ros2-nav2-stack)

# About ROS 2
> **Docs**: [docs.ros.org](https://docs.ros.org/)

- Robot Operating System (ROS) is NOT an OS
- Utilises but expands upon middleware functionality\*

=>

- Implements machine-to-machine data and control transfer
- Provides services for robotics systems

\* _Specifically, [ROS 2 uses DDS as its middleware](./info-docs/dds-in-ros2.md)._

---

ROS is mainly composed of 2 things:

- A core (middleware) with communication tools (e.g. DDS for ROS 2)
- A set of plug-and-play libraries and tools

> **Reference**: [_What is ROS?_, **RoboticsBackend.com**](https://roboticsbackend.com/what-is-ros/)

---

Essentially, it serves as the interface between:

- Computer OS
- Robotics hardware

=> Software libraries and tools for building robot applications:

- Device drivers (e.g. motor circuit board controllers)
- Algorithms (e.g. localisation and pathfinding)
- Developer tools (e.g. simulation and data visualisation)

---

**NOTE**: ROS has a number of the open source tools.

# About Navigation2 (Nav2)
> **Docs**: [docs.nav2.org](https://docs.nav2.org/)

A stack designed for robot navigation, built upon ROS 2.

---

**NOTE**: Stack = Collection of purpose-specific packages

# ROS 2 Humble
A distribution of ROS 2 that is:

- Well-established and stable (since 2022)
- Compatible with Nav2

For the above reasons, it has been chosen.

**NOTE**: _Learnings in one distribution are transferrable._

# ROS 2 installation (Humble)
[Ubuntu (deb packages), **docs.ros.org**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

# Nav2 stack installation and related installations
## Preliminary points
- `sudo apt update` (updates packages available in repositories)
- `sudo apt install ...` (installs specified packages)

## Packages to install
**NOTE**: Understanding what must be installed and why helps:

- Setup the entire ecosystem necessary for smooth operations
- Navigate installations for debugging/exploration

---

The Nav2 stack and its "bringup" system:

- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
    - A set of configurations, launch files, and scripts
    - Faciltates the startup and initialisation of Navigation2 <br> _For the given map/robots/hardware_

> **Read more on `nav2-bringup`**: [_Nav2-bringup_, **Qualcomm.com**](https://docs.qualcomm.com/bundle/publicresource/topics/80-70015-265/nav2-bringup_5_2_1.html)

# Other useful ROS 2 installations
## Turtlebot3
Mobile robot definitions with complete simulation (for testing):

`ros-humble-turtlebot3*`

**NOTE**: _`*` indicates that all Turtlebot3 dependencies must be installed._

## URDF Tutorial
Visualisation and easy customisation of URDF files (for robot modelling):

`ros-humble-urdf-tutorial`

## SLAM Toolbox
SLAM functionality for any robot:

`ros-humble-slam-toolbox`

---

**NOTE**: This package has a requirement:

_Laser scan must be published on the `/scan` topic._

# ROS 2 command line
Before running any ROS command, you must source the workspace.

> **References**:
>
> - ["Usage in enabling ROS 2 command line", Source Command in Linux](./source-command-in-linux.md#usage-in-enabling-ros-2-command-line)
> - ["5.2. Bring up the TurtleBot", _ROS 2 Turtlebot_, **ROS 2 Workshop**](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html#bring-up-the-turtlebot)

# Division of navigation-related operations
Navigation involves 2 key steps:

1. Map generation/definition; enables:
    - Global planning
    - Global pose estimation
2. Navigation; implements:
    - Global planning
    - Global pose estimation
    - Local planning (controller)
    - Local pose estimation

---

- SLAM is one way to deal with (1)
- Navigation2 stack is one way to deal with (2)

---

- Hence, SLAM is purely for mapping
- "Localisation" in SLAM is only for enabling accurate mapping
- Issues with SLAM result in issues with mapping <br> => Map data can become corrupted/inaccurate

Hence, a less computationally heavy\* and more stable approach:

- Perform SLAM or some other mapping technique carefully
- Thus, create a validated static map
- Use this a constant reference for the robot to perform:
    - Global path planning
    - Global pose estimation

\* _As SLAM requires intensive logic to append map data correctly._

---

**NOTE**: SLAM is not essential for navigation:

- SLAM can be replaced by any suitable mapping technique
- For use cases, we can even use existing/available map data <br> _Instead of building a new map via exploration_

_Navigation needs a static map, no matter the mapping technique._

# Further reading
## Linux and ROS 2-related
- [Source Command in Linux](./info-docs/source-command-in-linux.md) <br> _Also discusses_...
    - Setting up ROS 2 command line commands
    - Running ROS 2 nodes/packages
- [ROS Workspace](./info-docs/ros-workspace.md)
- [ROS Package](./info-docs/ros-package.md)
- [Build System](./info-docs/build-system.md) <br> _Also discusses_...
    - `colcon`, ROS 2's meta-build tool
    - `ament`, a family of Python CLI tools for ROS 2
- [ROS Bringup System](./info-docs/ros-bringup-system.md)
- [Middleware](./info-docs/middleware.md) (a core aspect of ROS)
- [DDS in ROS 2](./info-docs/dds-in-ros2.md) (DDS is ROS 2's middleware)
- [Transform in ROS 2](./info-docs/transform-in-ros2.md)

## Nav2-related
- [Nav2 Stack Overview](./info-docs/nav2-stack-overview.md)
- [Nav2 for Custom Robot](./info-docs/nav2-for-custom-robot.md)
- [Gazebo Simulation in Custom Environment Using Turtlebot3](./info-docs/gazebo-simulation-in-custom-environment-using-turtlebot3.md)
- [Interact Programmatically with Nav2](./info-docs/interact-programmatically-with-nav2.md)

## Procedure outlines
- [Navigation Steps Using Turtlebot3 Nav2](./info-docs/procedure-outlines/navigation-steps-using-turtlebot3-nav2.pdf)
- [SLAM Steps Using Turtlebot3](./info-docs/procedure-outlines/slam-steps-using-turtlebot3.pdf)

## Associated research
[The Marathon 2: A Navigation System](./associated-research/the-marathon-2--a-navigation-system.md)