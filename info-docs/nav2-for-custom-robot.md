<h1>NAV2 FOR CUSTOM ROBOT</h1>

---

**Contents**:

- [RECAP: Transforms (TFs) needed for Nav2](#recap-transforms-tfs-needed-for-nav2)
- [REQUIREMENT 1: Robot model](#requirement-1-robot-model)
- [REQUIREMENT 2: Input/output](#requirement-2-inputoutput)
  - [Odometry via multiple sensors](#odometry-via-multiple-sensors)
  - [Hardware controller](#hardware-controller)
- [General steps](#general-steps)

---

# RECAP: Transforms (TFs) needed for Nav2
1. `map --> odom`
2. `odom --> base_link` (via `base_footprint`)
3. `base_link --> base_scan`

> **Reference**: ["Key TFs needed for Nav2", _Nav2 Stack Overview_](./nav2-stack-overview.md#key-tfs-needed-for-nav2)

# REQUIREMENT 1: Robot model
**Robot modelling method**: Universal Robot Description File (URDF)

- Description of all elements in a robot
- Used by other packages to control/visualise robot accurately
- Given in XML format

**NOTE**: _URDF is a key input to enabling creation of TFs._

---

**NOTE**:

For many applications, implementing TFs is not necessary, because:

_ROS 2 packages may exist that use URDF to create TFs._

---

```
/joint_states --> robot_state_publisher --> /tf --> ...
                           ^
                           |
                          URDF
```

**NOTE**: _`robot_state_publisher` computes and publishes TFs._

# REQUIREMENT 2: Input/output
- Odometry (input to planner)
- Sensors (input to odometry)
- Controller (output to motors)

## Odometry via multiple sensors
```
WEn --> /odom --+                         +--> /odometry
IMU --> /Imu  --+--> robot_localization --+
... --> /...  --+                         +--> odom->base_link
```

**NOTE**:

- `WEn`: Wheel encoder
- `IMU`: Inertial Measurement Unit (IMU)
- `/odometry`: Is a topic
- `odom->base_link`: Is a TF

---

Minimum setup for navigation:

- Wheel encoder
- LiDAR (topic: `/scan`, message: `sensor_msgs/msg/LaserScan`)

---

**NOTE**: ROS 2 package exist to...

- process
- publish

... data from sensors compatible with ROS 2.

## Hardware controller
- Subscribes to `/cmd_vel` topic
- Uses message: [`geometry_msgs/Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html); breaks velocity into:
    - Linear velocity
    - Angular velocity
- Actuates motors accordingly

# General steps
> **Further detail**: [_SLAM and Navigation Steps for Any Robot_, `procedure-outlines`](./procedure-outlines/slam-and-navigation-steps-for-any-robot.pdf)

---

**1. Run the [stack](./definitions.md#stack) for the robot**:

_Initialising a robot you can control_...

  - Run robot-specific nodes <br> E.g.: _Sensor fusion, motor actuation, etc._
  - Start publishing required data on appropriate topics <br> E.g.: _Sensor data (e.g. laser scan), odometry, etc._
  - **NOTE 1**: By itself, this does not involve SLAM/navigation
  - **NOTE 2**: The stack can be for physical robot or simulation

E.g.: Using Turtlebot3 (but you can use any robot stack):

`ros2 launch turtlebox_gazebo ...`

Replace `...` with launch file for desired simulated environment.

**Turtlebot3-specific requirement**:

Specify the desired Turtlebot3 URDF model to be used:

This is done by exporting the environment variable `TURTLEBOT3_MODEL` as follows:

```
export TURTLEBOT3_MODEL=... # replace ... with a valid URDF model name, e.g. "waffle"
```

Not doing this leads to the following exception when launching Turtlebot3 Gazebo simulations:

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 'TURTLEBOT3_MODEL'
```

**NOTE**: *Place this command in `~/.bashrc` to ensure this variable is exported for every terminal session.*

**KEY POINT**:

- To allow for SLAM and obstacle avoidance, ensure: <br> `LaserScan` message is being piublished in the `/scan` topic

---

**2. Launch the Navigation2 stack**:

CASE 1: _If you intend to operate SLAM_...

`ros2 launch nav2_bringup navigation_launch.py`

Optional arguments:

- `use_sim_time:=True` (if using simulation)

CASE 2: _If you do not intend to operate SLAM_...

`ros2 launch nav2_bringup bringup_launch.py`

Optional arguments:

- `use_sim_time:=True` (if using simulation)
- `map:=...` (if using generated map; replace `...` with path) <br> **NOTE**: _Reference the generated map's YAML file_

---

(Optional: If you intend to operate SLAM)

**3. Launch SLAM toolbox**:

`ros2 launch slam_toolbox online_async_launch.py`

Optional arguments:

- `use_sim_time:=True` (if using simulation)

---

**4. Run RViz2 for visualisation and GUI-based inputs**:

`ros2 run rviz2 rviz2`

Within the RViz window, select the necessary parameters to display:

- Map (ensure also to set its topic field as `/map`)\* <br> _This topic only runs upon launching Navigation2_
- TF (transforms; allows visualising different robot frames)
- LaserScan
- RobotModel (NON-ESSENTIAL; allows viewing robot model) <br> _To enable the model_...
    - Set "Description Source" field to "Topic"
    - Set "Description Topic" field to `/robot_description`

If operating only navigation and not SLAM, get costmaps as follows:

- Add another "Map" section; select the topic as `/global_costmap`
- Add another "Map" section; select the topic as `/local_costmap`

**NOTE**: _Subscribe to the respective topics in above sections._

\* _In case of `WARNING: No map received`_...

- Go the the "Durability" field in the "Map" parameter section
- Change the value from "Volatile" to "Transient Local"