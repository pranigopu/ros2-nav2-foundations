<h1>ROS 2 Nav2 Foundations</h1>

---

**Contents**:

- [About ROS 2](#about-ros-2)
- [About Navigation2 (Nav2)](#about-navigation2-nav2)
- [About ROS 2 Humble](#about-ros-2-humble)
- [Installation and setup](#installation-and-setup)
- [Division of navigation-related operations](#division-of-navigation-related-operations)
- [Further reading](#further-reading)
  - [Linux and ROS 2-related](#linux-and-ros-2-related)
  - [Nav2-related](#nav2-related)
  - [Procedure outlines](#procedure-outlines)
  - [Associated research](#associated-research)

---

> **Reference course**: [_ROS 2 Nav2 [Navigation 2 Stack] - with SLAM and Navigation_, **udemy.com**](https://www.udemy.com/course/ros2-nav2-stack)

---

**Abbreviations**:

- RMW = ROS Middleware
- URDF = Unified Robot Description Format

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

# About ROS 2 Humble
A distribution of ROS 2 that is:

- Well-established and stable (since 2022)
- Compatible with Nav2

For the above reasons, it has been chosen.

**NOTE**: *Learnings in one distribution are transferrable.*

# Installation and setup
See [`info-docs`/`installation-and-setup`](./info-docs/installation-and-setup/)

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

\* *As SLAM requires intensive logic to append map data correctly.*

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
- [ROS Core Concepts](./info-docs/ros-core-concepts.md)
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
- [Nav2 for Custom Robot and Custom Map](./info-docs/nav2-for-custom-robot-and-custom-map.md)
- [Gazebo Simulation in Custom Environment Using Turtlebot3](./info-docs/gazebo-simulation-in-custom-environment-using-turtlebot3.md)
- [Explore Nav2-Related Topics and Actions](./info-docs/nav2-related-topics-and-actions.md)
- [Interact Programmatically with Nav2](./info-docs/interact-programmatically-with-nav2/)
- [Nav2 Parameter Configuration](./info-docs/nav2_param_config.md)

## Procedure outlines
- [Navigation Steps Using Turtlebot3 Nav2](./info-docs/procedure-outlines/navigation-steps-using-turtlebot3-nav2.pdf)
- [SLAM Steps Using Turtlebot3](./info-docs/procedure-outlines/slam-steps-using-turtlebot3.pdf)

## Associated research
[The Marathon 2: A Navigation System](./associated-research/the-marathon-2--a-navigation-system.md)