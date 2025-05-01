<h1>INTERACT PROGRAMMATICALLY WITH NAV2</h1>

---

**Contents**:

- [Exploring Nav2-related topics and actions](#exploring-nav2-related-topics-and-actions)
- [Nav2 simple commander API](#nav2-simple-commander-api)
	- [Purpose](#purpose)
	- [Installation](#installation)
	- [Installations for facilitating custom Python scripting](#installations-for-facilitating-custom-python-scripting)

---

# Exploring Nav2-related topics and actions
The following topics and actions were explored with the following running processes:

**Terminal 1: Gazebo simulation**:

```sh
ros2 launch turtlebot3_gazebo turtlebot3_design_lab.launch.py
```

**Terminal 2: Navigation2 bringup launcher**:

```sh
cd ~/maps # Contains .pgm and .yaml files for LiDAR-generated environment maps
ros2 launch nav2_bringup bringup.launch.py use_sim_time:=True map:=design-lab-via-slam.yaml
```

**Terminal 3: RViz2 for visualisation**:

```
ros2 run rviz2 rviz2
```

---

The following data were gathered in terminal 4:

**Information on `/initialpose` topic**:

```
$ ros2 topic info /initialpose

Type: geometry_msgs/msg/PoseWithCovarianceStamped
Publisher count: 1
Subscription count: 1
```

**Information on available actions**:

```
$ ros2 action list

/assisted_teleop
/backup
/compute_path_through_poses
/compute_path_to_pose
/drive_on_heading
/follow_path
/follow_waypoints
/navigate_through_poses
/navigate_to_pose
/smooth_path
/spin
/wait
```

**Information on `navigate_to_pose` action**:

```
$ ros2 action info /navigate_to_pose

Action: /navigate_to_pose
Action clients: 4
    /bt_navigator
    /waypoint_follower
    /rviz
    /rviz_navigation_dialog_action_client
Action servers: 1
    /bt_navigator
```

# Nav2 simple commander API
## Purpose
Facilitates the process of (programmatically):

- Creating publishers for certain topics (e.g. `/initialpose`)
- Performing/executing actions (e.g. `navigate_to_pose`)

*Acts as an interface between custom programs and Nav2 stack.*

## Installation
```
sudo apt install ros-<distro>-nav2-simple-commander # Replace <distro> with the name of your ROS 2 distribution
```

As I am using ROS 2 Humble:

```
sudo apt install ros-humble-nav2-simple-commander
```

**NOTE**: *It should have been installed when installing Nav2.*

## Installations for facilitating custom Python scripting
### `tf-transformations`
- Reimplementation of `tf/transformations.py` library
- For common Python spatial operations (e.g. Euler-quaternion conversions)
 
> **Reference**: [`tf_transformations`, **docs.ros.org**](https://docs.ros.org/en/ros2_packages/humble/api/tf_transformations/)

Install using:

```
sudo apt install ros-humble-tf-transformations # For ROS 2 Humble distribution
```

### `python3-transforms3d`
Code to convert between various geometric transformations:

- Composing rotations / zooms / shears / translations into affine matrix
- Decomposing affine matrix into rotations/zooms/shears/translations
- Conversions between different representations of rotations, including:
	- 3x3 Rotation matrices
	- Euler angles
	- Quaternions

> **Reference**: [`transforms3d 0.4.2`, **pypi.org**](https://pypi.org/project/transforms3d/)

**SIDE NOTE**: **"Affinity" is a geometric transformation that preserves lines and parallelism.*

Install using:

```
sudo apt install python3-transforms3d
```

Alternatively, you can do:

```
pip install transforms3d
```