<h1>BUILD SYSTEM</h1>

---

**Contents**:

- [Build system](#build-system)
  - [Framework](#framework)
  - [Specifications](#specifications)
- [Build tool](#build-tool)
- [Build helpers](#build-helpers)
  - [Definition](#definition)
  - [Ament](#ament)
- [Ament package](#ament-package)
- [Meta-build tool](#meta-build-tool)
  - [Definition](#definition-1)
  - [`colcon`](#colcon)

---

# Build system
A directory structure and procedure to build ROS 2 code as needed.

---

Consists of:

- [Framework](#framework) for identifying key build components
- [Specifications](#specifications) for correct build creation

## Framework
- ROS 2 relies heavily on the division of code into packages
- Each package contains a **manifest file** (package.xml)

## Specifications
The **manifest file**:

- Contains essential metadata about the package, e.g.:
    - Build type
    - Dependencies
- Is required for [meta-build tool](#meta-build-tool) to function

# Build tool
Software controling compilation and testing of a single package.

---

Common build tools in ROS 2:

- CMake for C++
- `setuptools` for Python

_Some other build tools are also supported._

# Build helpers
## Definition
Helper functions that hook into build tool.

_Their purpose is to improve developer experience during building._

## Ament
A family of CLI Python tools used for software dev with ROS 2.

---

**NOTE**:

- Ament tools can be used from any build system
- ROS 2 packages often rely on Ament tools for build helpers

---

> **References**:
>
> - [Ament Lint CLI Utilities, **docs.ros.org**](https://docs.ros.org/en/rolling/Tutorials/Advanced/Ament-Lint-For-Clean-Code.html)
> - [The build system, **docs.ros.org**](https://docs.ros.org/en/rolling/Concepts/Advanced/About-Build-System.html)

# Ament package
Any package which:

- Contains a `package.xml` manifest file
- Follows the packaging guidelines of Ament

---

The above are regardless of the underlying build system

=> Any ROS 2 package can be an Ament package

---

> **Reference**: [The build system, **docs.ros.org**](https://docs.ros.org/en/rolling/Concepts/Advanced/About-Build-System.html)

# Meta-build tool
## Definition
A piece of software that defines the order package building

=> Build or test them in the correct dependency order

---

It calls the build tool to do the actual work of:

- Compiling the package
- Testing the package
- Installing the package

## `colcon`
_`colcon` => "Collective Construction"_

In ROS 2, the tool **colcon** is used as the meta-build tool.

**NOTE**: _Its code is open source._

---

`colcon` is a command line tool to improve the workflow of:

- Building multiple software packages
- Testing multiple software packages
- Using multiple software packages

It automates the process by:

- Handling the ordering of package building/testing
- Setting up the environment to use the packages

---

**Reference**: [colcon - collective construction (official doc)](https://colcon.readthedocs.io/en/released/)

---

**Installing `colcon` (Linux)**:

```sh
# Adding the ROS 2 Ubuntu repository in the list of sources to access relevant packages for installation:
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
# Obtaining the key that ensures the the material in the ROS 2 repositories are authentic:
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# NOTE:
# - The above are not necessary to do if you have already done ROS 2 binary installations
# - This is because the above would have already been done during those installations

# Updating the source repositories (to account for new packages/modifications) and upgrading existing installations: 
sudo apt update && sudo apt upgrade

# Installing colcon
sudo apt install python3-colcon-core # Enables core functionality
sudo apt install python3-colcon-common-extensions # Enables compilation progress visual display, among other things
```