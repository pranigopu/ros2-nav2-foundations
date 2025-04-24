<h1><code>source</code> COMMAND IN LINUX</h1>

---

**Contents**:

- [About the `source` command](#about-the-source-command)
- [Inclusion of source commands in `~/.bashrc`](#inclusion-of-source-commands-in-bashrc)
  - [Usage in enabling ROS 2 command line](#usage-in-enabling-ros-2-command-line)
- [Usage in running ROS 2 package installations](#usage-in-running-ros-2-package-installations)
- [Usage of `source` to run `~/.bashrc`](#usage-of-source-to-run-bashrc)

---

# About the `source` command
- A built-in feature of the Linux shell
- Designed to execute instructions stored within a file
- Executes the above directly in the current shell environment

---

Hence, when you use the `source` command, it:

- Reads contents of specified file (e.g. sequence of instructions)
- Executes them as if they were typed directly into the terminal

---

**Reference**: [_source Command in Linux with Examples_, **GeeksForGeeks.org**](https://www.geeksforgeeks.org/source-command-in-linux-with-examples/)

# Inclusion of source commands in `~/.bashrc`
The `.bashrc` file is a script file that:

- Is executed when a user begins a terminal session
- Contains a series of configurations for the terminal session
- Includes setting up or enabling:
    - Coloring
    - Completion
    - Shell history
    - Command aliases
    - etc.

> **Reference**: [_What is .bashrc file in Linux?_, **DigitalOcean.com**](https://www.digitalocean.com/community/tutorials/bashrc-file-in-linux)

---

Including a source command at the end of this file ensures:

- The ROS 2 package(s) and its node(s) are installed in the session
- The above can be accessed by `ros2 run` or `ros2 launch` commands <br> _`run` for nodes, `launch` for whole packages_

## Usage in enabling ROS 2 command line
_The ROS 2 command line tool is referred to as `ros2`._

- `ros2` consists of a number of commands for operating on ROS 2
- Installing these commands in the terminal session requires setup
- This setup script is defined in the ROS 2 distribution directory

E.g.: `/opt/ros/humble/setup.bash`

---

**NOTE**:

- There are other setup scripts present apart from `.bash`
- Use the one that works (e.g. shell script `.sh`, etc.)

---

One way to run this script is:

```bash
source /opt/ros/humble/setup.bash
```

Another way is to include the above command in `~/.bashrc`.

_The latter automatically sets up `ros2` every terminal session._

# Usage in running ROS 2 package installations
After building ROS 2 packages in a workspace, we get:

```c
<workspace name>
    - build // Contains build code/programs and specifications
    - install // Contains installation setup script
    - logs // Contains build logs
    - src
        - <package 1>
        - <package 2>
        ...
```

---

- `install` contains `setup.bash`
- This is the installation setup script for the built code
- This code must be sourced to:
    - Install the code within the current shell environment
    - Access the package and its nodes directly via CLI

Hence, do:

```bash
source < path >/install/setup.bash
```

# Usage of `source` to run `~/.bashrc`
- `~/.bashrc` runs at the start of a new terminal session
- However, it can be edited within a session
- To apply changes in the same session, do `source ~/.bashrc`