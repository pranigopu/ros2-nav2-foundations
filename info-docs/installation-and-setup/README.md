<h1>INSTALLATION AND SETUP (ROS 2 and NAV2)</h1>

---

**Contents**:

- [ROS 2 Humble installation and environment setup](#ros-2-humble-installation-and-environment-setup)
	- [Binary installation](#binary-installation)
	- [Building from source](#building-from-source)
	- [Compatible OS for ROS Humble installation](#compatible-os-for-ros-humble-installation)
- [Suggested DDS implementation for Nav2](#suggested-dds-implementation-for-nav2)
- [Installing relevant ROS 2 packages](#installing-relevant-ros-2-packages)
- [ROS 2 command line](#ros-2-command-line)

---

# ROS 2 Humble installation and environment setup
## Binary installation
For Ubuntu-specific ROS 2 as binary packages + environment setup:

[*Ubuntu (deb packages)*, **docs.ros.org/en/humble**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

"Desktop install", done using:

```
sudo apt install ros-humble-desktop
```

WHY INSTALL THIS: *Comes bundled with ROS, RViz, demos and tutorials.*

---

**Shell script to automate installation:**

[ROS Humble Desktop Installation - Shell Script](../../scripts/installation-and-setup/ros-humble-desktop-installation.sh)

***For Linux...***

- To give execution permission, do `chmod +x ...`
- To run, do `./...`

**NOTE**: *Replace `...` with script path.*

## Building from source
Building ROS from source is done using the steps outlined here:

[Build ROS 2 Humble from Source](./build-ros2-humble-from-source.md)

---

**Shell script to partially automate setup**:

[ROS Humble Build from Source - Shell Script](../../scripts/installation-and-setup/ros-humble-build-from-source.sh)

***For Linux...***

- To give execution permission, do `chmod +x ...`
- To run, do `./...`

**NOTE**: *Replace `...` with script path.*

## Compatible OS for ROS Humble installation
As of 2025-04-29:

ROS 2 Humble binary installations are supported only for:

- Ubuntu Linux 22.04
- Windows 10
- RHEL-8

ROS 2 Humble building from source is supported only for the above + macOS.

---

Hence, if using Ubuntu, it is key to use the version:

22.04 (code-named "Jammy")

The latest version, "Noble" (24.04) does not yet support ROS 2 Humble installations.

> **Reference**: [*Installation*, **docs.ros.org/en/humble**](https://docs.ros.org/en/humble/Installation.html)

# Suggested DDS implementation for Nav2
**Eclipse Cyclone DDS**

Is a very performant and robust open-source DDS implementation.

> **Reference**: [*Eclipse Cyclone DDS*, **docs.ros.org/en/humble**](https://docs.ros.org/en/humble/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

---

**Rationale**:

- [*Fast-DDS Service Reliability sometimes hangs lifecycle manager*, **github.com/ros-navigation/navigation2/issues**](https://github.com/ros-navigation/navigation2/issues/3033) <br> *According to this source*...
	- Nav2's lifecycle manager can block indefinitely using FastDDS
	- This issue never occurs with CycloneDDS
- ["Quick fix and DDS issue with Nav2", *ROS2 Nav2 Tutorial*, **RoboticsBackend.com**](https://roboticsbackend.com/ros2-nav2-tutorial/#Quick_fix_and_DDS_issue_with_Nav2) <br> *According to this source*...
	- ROS 2's default Fast DDS, does not work well with Nav2
	- Cyclone DDS has been suggested as a fix

---

**Installation and switching RMW implementation**:

[*Eclipse Cyclone DDS*, **docs.ros.org/en/humble**](https://docs.ros.org/en/humble/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

**NOTE 1**: *I installed the packages rather than building from source.*

**NOTE 2**: *`rosdep` is useful only if building from source.*

# Installing relevant ROS 2 packages
**Preliminary points**:

- `sudo apt update` (updates packages available in repositories)
- `sudo apt install ...` (installs specified packages)

**NOTE**: Understanding what must be installed and why helps:

- Setup the entire ecosystem necessary for smooth operations
- Navigate installations for debugging/exploration

---

**Nav2 stack for navigation**:

The Nav2 stack and its "bringup" system:

- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
    - A set of configurations, launch files, and scripts
    - Faciltates the startup and initialisation of Navigation2 <br> *For the given map/robots/hardware*

> **Read more on `nav2-bringup`**: [*Nav2-bringup*, **Qualcomm.com**](https://docs.qualcomm.com/bundle/publicresource/topics/80-70015-265/nav2-bringup_5_2_1.html)

---

**Turtlebot3 for ready-made simulation**:

Mobile robot definitions with complete simulation (for testing):

`ros-humble-turtlebot3*`

**NOTE**: *`*` indicates that all packages with names starting with `ros-humble-turtlebot3` must be installed.*

---

**URDF Tutorial for dealing with URDF files**:

Visualisation and easy customisation of URDF files (for robot modelling):

`ros-humble-urdf-tutorial`

---

**SLAM Toolbox for SLAM functionality**:

SLAM functionality for any robot:

`ros-humble-slam-toolbox`

---

**NOTE**: This package has a requirement:

*Laser scan must be published on the `/scan` topic.*

# ROS 2 command line
Before running any ROS command, you must source the workspace.

> **References**:
>
> - ["Usage in enabling ROS 2 command line", Source Command in Linux](./source-command-in-linux.md#usage-in-enabling-ros-2-command-line)
> - ["5.2. Bring up the TurtleBot", *ROS 2 Turtlebot*, **ROS 2 Workshop**](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html#bring-up-the-turtlebot)
