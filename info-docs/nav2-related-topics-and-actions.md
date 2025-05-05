<h1>EXPLORE NAV2-RELATED TOPICS AND ACTIONS</h1>

---

**Contents**:

- [Topics and actions in ROS 2](#topics-and-actions-in-ros-2)
- [Practical exploration](#practical-exploration)

---

# Topics and actions in ROS 2
See the relevant headings in [ROS Core Concepts](./ros-core-concepts.md).

# Practical exploration
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