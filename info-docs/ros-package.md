<h1>ROS 2 PACKAGE</h1>

---

**Contents**:

- [About](#about)
- [Basic approach](#basic-approach)
- [Key optional arguments in package creation](#key-optional-arguments-in-package-creation)
  - [`--build-type`](#--build-type)
  - [`--dependencies`](#--dependencies)

---

# About
Packages allow organising nodes and dependencies.

**NOTE**: _Package => Collection of purpose-specific programs_

# Basic approach
To define a package in the workspace:

- Create a subdirectory called `src`
- In this, run the command: `ROS 2 pkg create <package name> ...`

The entire workspace is built by building nodes written in `src`.

# Key optional arguments in package creation
## `--build-type`
**RELEVANCE**:

- The build tool uses a build system
- A build system is a means of specifying:
    - Build programming stack
    - Key build configurations

Hence, when creating a ROS 2 package:

_Specify the build type_

=>

_Ensure the right build system is set up for the package_

---

`--build-type` specifies the built system type; some options:

- `ament_python` => Python stack
- `ament_cmake` => C++ stack

---

For more on "Ament" in the context of ROS 2 packages, see:

["Ament package" from `build-system.md`](./build-system.md#ament-package)

## `--dependencies`
Specifies dependencies used by the package.

**NOTE**: Dependencies can be:

- Global installations
- Other packages (ROS 2 or otherwise)

---

For ROS 2 node functionality, use the dependency `rclpy`.