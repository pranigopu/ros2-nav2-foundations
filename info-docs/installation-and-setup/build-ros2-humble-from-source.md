<h1>BUILD ROS 2 HUMBLE FROM SOURCE</h1>

---

**Contents**:

- [Installation setup](#installation-setup)
- [Issues](#issues)
- [SPECIAL NOTE: Installing Orocos KDL](#special-note-installing-orocos-kdl)
	- [What is KDL?](#what-is-kdl)
	- [ROS 2 dependency on KDL](#ros2-dependency-on-kdl)
	- [Installing Orocos KDL locally](#installing-orocos-kdl-locally)
	
---

# Installation setup
> **Reference**: [*Ubuntu (source)*, **docs.ros.org/en/humble**](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

For details and additional points, see [`ros-humble-build-from-source.sh`](./ros-humble-build-from-source.sh).

# Issues
["Issues when trying to build ROS Humble from source", `issue-log.md`](./issue-log.md#issues-when-trying-to-build-ros-humble-from-source)

# SPECIAL NOTE: Installing Orocos KDL
KDL = Kinematics and Dynamics Library

## What is KDL?
KDL is an *application independent* (i.e. general purpose) framework for:

- Mdelling kinematic chains
- Computing movement and forces of kinematic chains

Examples of kinematic chains:

- Robots
- Biomechanical human models
- Computer-animated figures
- Machine tools

KDL provides class libraries for:

- Geometrical objects (point, frame, line,... )
- Kinematic chains of various families (serial, humanoid, parallel, mobile,... )
- Motion specification and interpolation of the above

> **Reference**: [*KDL Wiki \| The Orocos Project*, **Orocos Kinematics and Dynamics**](https://www.orocos.org/kdl.html)

---

**KEY CONCEPT: KDL tree**:

KDL defines a tree structure to represent the...

- kinematic parameters
- dynamic parameters

... of a robot mechanism.

> **Reference**: [`kdl_parser`, **docs.ros.org**](https://docs.ros.org/en/ros2_packages/rolling/api/kdl_parser/)

## ROS 2 dependency on KDL
ROS 2 builds that are built from source utilise KDL for the following packages:

- `kdl_parser` <br> *Provides tools to construct KDL tree from XML robot representation in URDF*
- `tf2_geometry_msgs/tf2_eigen_kdl`
- `tf2_geometry_msgs/tf2_kdl`
- `robot_state_publisher`

**NOTE**:

- Orocos KDL was a core part of ROS 1, but not necessarily ROS 2
- Orocos KDL is an optional package for ROS 2 binary installations
- ROS 2 has other packages to do similar functionality
- However, Orocos KDL has proven to be a key package when building from source

## Installing Orocos KDL locally
**Via ROS 2 binary installation**::

An easy way would be as follows:

- Locally obtain and source a binary installation of ROS 2
- Install Orocos's toolchain using `sudo apt install ros-${ROS_DISTRO}-orocos-toolchain`
- Install Orocos KDL using `sudo apt install ros-${ROS_DISTRO}-orocos-kdl`

> **Reference**: [*Requirements and installation guide*, **docs.orocos.org**](https://docs.orocos.org/docs_main/installation.html)

However, this involves installing and sourcing binary ROS 2 installations:

- This can affect the sourcing and usage of ROS 2 builds from source
- This prevents us from knowing how to actually build ROS 2 purely from source
- This knowledge-gap could be detrimental if we need to optimise disk storage <br> E.g.: *In real-life applications*

---

**Via building from source**:

1.<br>
Within a desired workspace, clone `orocos_toolchain` repo:

```
git clone --recursive https://github.com/orocos-toolchain/orocos_toolchain.git
```

Now, you will get `orocos_toolchain` within your workspace.

**NOTE**: *`--recursive` option is used to update\* submodules recursively.*

2.<br>
Set the correct GCC and G++ versions.

REASONS:

- ["Improper `g++` version", `issue-log.md`](./issue-log.md#improper-g-version)
- ["Improper `gcc` version", `issue-log.md`](./issue-log.md#improper-gcc-version)

PROCEDURE:

Given in the above 2 issue log sections.

3.<br>
Run the configuration `configure` file within `orocos_toolchain` as follows:

```
chmod +x configure # Gives the file execution permissions
./configure # Runs the file
```

4.<br>
Run `ccmake .` within `orocos_toolchain` and toggle `ENABLE_COBRA` to `OFF`.

REASON:

["Missing COBRA implementation", `issue-log.md`](./issue-log#missing-cobra-implementation)

PROCEDURE:

Given in the above issue log section.

5.<br>
Run `make install` within `orocos_toolchain`.

6.<br>
Source the `setup.sh` file of `orocos_toolchain` to set it up in the environment:

```
source .../orocos_toolchain/setup.sh
```

*Replace `...` with the workspace's path.*

**NOTE**: *To ensure it has execution permissions, do `chmod -x setup.sh`.

To set it up for every terminal session, add the following to `~/.bashrc`:

```
chmod -x setup.sh
source .../orocos_toolchain/setup.sh 
```

*Replace `...` with the workspace's path.*