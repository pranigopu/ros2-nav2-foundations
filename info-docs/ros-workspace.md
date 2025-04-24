<h1>ROS 2 WORKSPACE</h1>

---

**Contents**:

- [About](#about)
- [Conventions for workspace definition](#conventions-for-workspace-definition)
- [Directory structure](#directory-structure)

---

# About
A ROS 2 workspace is a directory in which ROS 2 packages can be:

- Defined (as source code)
- Built (as compiled or executable code)
- Prepared for installation (via shell/bash setup scripts)

**NOTE**: _A ROS 2 workspace follows a certain directory structure._

# Conventions for workspace definition
See [_Naming Conventions for Catkin Based Workspaces_, **ros.org**](https://ros.org/reps/rep-0128.html).

# Directory structure
Before building any packages:

```c
<workspace name>
    - src
        - <package 1>
        - <package 2>
        ...
```

After building packages:

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