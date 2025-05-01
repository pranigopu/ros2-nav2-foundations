<h1>GAZEBO SIMULATION IN CUSTOM ENVIRONMENT</h1>

**_Using Turtlebot3_**

---

**Contents**:

- [Broad steps](#broad-steps)
- [Creating a custom Turtlebot3 workspace](#creating-a-custom-turtlebot3-workspace)
- [Adding required files in custom Turtlebot3 workspace](#adding-required-files-in-custom-turtlebot3-workspace)

---

# Broad steps
- Designing environment in Gazebo
- Saving world model
- Creating a custom Turtlebot3 workspace\*
- Adding required files in custom Turtlebot3 workspace

\* _To add new source files to launch custom world._

# Creating a custom Turtlebot3 workspace
We need to add and build custom programs and files for:

- Custom world model
- Launch file for starting Turtlebot3 in the custom world

---

Now, we cannot edit ROS 2 Humble installations; they are read-only.

---

Hence, we must do the following...

---

**1. Create a ROS workspace for the custom Turtlebot3 build**:

E.g.: `~/my_turtlebot3_workspace`

In this workspace directory, define the `src` directory:

`~/my_turtlebot3_workspace/src`

This is where the source code will go.

> **For more on ROS workspaces**: [_ROS Workspace_](./ros-workspace.md)

---

**2. Clone the Turtlebot3 simulations GitHub repository**:

Link: [`github.com`/`ROBOTIS-GIT`/`turtlebot3_simulations`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 

You must clone it in the `src` directory defined above.

An easy way to do this is navigate to `src` and do:

```
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations .
```

**NOTE**: _The final dot refers to the current directory `src`._

**IMPORTANT SUB-STEP**:

The `turtlebot3_simulations` repo does not have all the dependencies needed by Turtlebot3 simulation packages, e.g. ROS 2 Gazebo-related packages to help run robotics simulations. Not having these dependencies means the custom Turtlebot3 simulation packages cannot be built due to compilation errors (due to missing dependencies). To ensure that these dependencies are available, first obtain the binary installations of Turtlebot3-related packages compatible with the ROS 2 distribution being used; in my case, as I am using ROS 2 Humble, this step looks like:

```
sudo apt install ros-humble-turtlebot3* # * means all packages with names starting with "ros-humble-turtlebot3" must be installed
```

This step automatically installs the necessary dependencies.

*Make sure to source this installation; doing so is not needed if you are already sourcing your binary ROS 2 installations.*

---

**3. Switch the the `humble` branch**:

(_Assuming our ROS 2 distribution is Humble_)

Within `src`, do:

```
git checkout humble
```

This switches the repository branch from `main` to `humble`.

***Why is it necessary?***

- To ensure source files are compatible with ROS 2 Humble
- To ensure already-installed ROS 2 Humble binary installations can be used as dependencies

*Not doing this could lead to compilation errors.*

---

**4. Add custom files/source code where necessary**:

See ["Adding required files in custom Turtlebot3 workspace"](#adding-required-files-in-custom-turtlebot3-workspace).

---

**5. Build packages from `src`**:

In `~/my_turtlebot3_workspace`, run:

```
colcon build
```

This builds the packages defined in `src`.

> **For more on ROS build system**: [_Build System_](./build-system.md)

---

**6. Source the installation setup script for the build**:

<u>OPTION 1</u>: Source in current terminal session:

```
source ~/my_turtlebot3_workspace/install/setup.bash
```

Overlays existing Turtlebot3 installation in current session.

<u>OPTION 2</u>: Source in `.bashrc`:

- Open `.bashrc` using `gedit ~/.bashrc` <br> _Or any other text editor other than `gedit`, e.g. `nano`_
- Below the source command for ROS 2 installation, add: <br> `source ~/my_turtlebot3_workspace/install/setup.bash`
    - Overrides existing Turtlebot3 installation in current session
    - Hence, we enable the use of our custom Turtlebot3 build
- Run `source ~/.bashrc` to apply changes in current session <br> _For new sessions, this file is automatically sourced_

> **For more on source command in Linux**: [_Source Command in Linux_](./source-command-in-linux.md)

---

**7. Specify the desired Turtlebot3 URDF model to be used**:

This is done by exporting the environment variable `TURTLEBOT3_MODEL` as follows:

```
export TURTLEBOT3_MODEL=... # replace ... with a valid URDF model name, e.g. "waffle"
```

Not doing this leads to the following exception when launching Turtlebot3 Gazebo simulations:

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 'TURTLEBOT3_MODEL'
```

**NOTE**: *Place this command in `~/.bashrc` to ensure this variable is exported for every terminal session.*

# Adding required files in custom Turtlebot3 workspace
**NOTE**: _Do this before building the custom Turtlebot3 packages._

**1. Add the custom world model in**:

```
~/my_turtlebot3_workspace/src/turtlebot3_gazebo/worlds
```

**NOTE**: The world model is as follows:

- `.world` extension
- Essentially an XML file
- Any valid file name will do

---

**2. Add world launch file in**:

```
~/my_turtlebot3_workspace/src/turtlebot3_gazebo/launch
```

**NOTE**: This launch file is as follows:

- Python source file (`.py` extension)
- Generates world and spawns robot model in Gazebo simulation

**EXAMPLE**:

```python
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#############################################################
# FUNCTION TO GENERATE LAUNCH DESCRIPTION
#############################################################

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_design_lab.world'
    )

    #================================================
    # DEFINING LAUNCH COMMANDS

    #------------------------------------
    # GAZEBO SERVER: Runs the Gazebo simulation engine (includes physics engine):
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    #------------------------------------
    # GAZEBO CLIENT: Runs the Gazebo simulation window to view and control the simulation:
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    #------------------------------------
    # ROBOT STATE PUBLISHER: Computes and publishes transforms related to the robot's state:
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #------------------------------------
    # SPAWN TURTLEBOT: Spaws a Turtlebot3 model (the model name/path is given by the terminal session's environment variable TURTLEBOT3_MODEL):
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    #------------------------------------
    # Add the commands to the launch description:
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
```