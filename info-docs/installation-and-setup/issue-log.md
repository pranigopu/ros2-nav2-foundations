<h1>ISSUE LOG</h1>

Log detailing issues and solutions revolving around:

- Installation procedures
- Broken/missing dependencies
- Installation incompatibilities

---

**Contents**:

- [Incompatible OS for ROS Humble installations](#incompatible-os-for-ros-humble-installations)
- [RViz unable to run due to display issues with Qt](#rviz-unable-to-run-due-to-display-issues-with-qt)
- [RViz unresponsive to mouse clicks](#rviz-unresponsive-to-mouse-clicks)
- [Issues when trying to build ROS Humble from source](#issues-when-trying-to-build-ros-humble-from-source)
	- [Unable to compile ROS 2 `iceoryx_hoofs` package due to missing header file](#unable-to-compile-ros-2-iceoryx_hoofs-package-due-to-missing-header-file)
	- [Unable to compile ROS 2 `tinyxml2_vendor` package due to missing dependencies](#unable-to-compile-ros-2-tinyxml2_vendor-package-due-to-missing-dependencies)
	- [Unable to compile ROS 2 `libcurl_vendor` package due to missing dependencies](#unable-to-compile-ros-2-libcurl_vendor-package-due-to-missing-dependencies)
	- [Unable to compile ROS 2 `fastrtps` package due to missing Asio installation](#unable-to-compile-ros-2-fastrtps-package-due-to-missing-asio-installation)
	- [Unable to compile ROS 2 `orocos_kdl_vendor` package](#unable-to-compile-ros-2-orocos_kdl_vendor-package)
		- [Related error with `kdl_parser` package](#related-error-with-kdl_parser-package)
		- [Related error with `tf2_eigen_kdl` package](#related-error-with-tf2_eigen_kdl-package)
		- [Related error with `tf2_kdl` package](#related-error-with-tf2_kdl-package)
	- [Issues in installing Orocos KDL locally](#issues-in-installing-orocos-kdl-locally)
		- [Improper `g++` version](#improper-g-version)
		- [Improper `gcc` version](#improper-gcc-version)
		- [Missing COBRA implementation](#missing-cobra-implementation)
		- [Broken `orocos_toolchain` build](#broken-orocos_toolchain-build)
	- [Package build failing due to memory-hogging](#package-build-failing-due-to-memory-hogging)
		- [Solving issues in building `rviz_default_plugins` by increasing swap space](#solving-issues-in-building-rviz_default_plugins-by-increasing-swap-space)
	- [C++ compilation issues for ROS 2 source files](#c-compilation-issues-for-ros-2-source-files)
	- [RViz unable to load default plugins despite them being built](#rviz-unable-to-load-default-plugins-despite-them-being-built)
	- [Unresolved error](#unresolved-error)

---

# Incompatible OS for ROS Humble installations
2025-04-29

- Tried installing ROS Humble binaries for Ubuntu 24.04
- All necessary repositories were sourced
- No packages were found with the prefix `ros-humble-`

---

**Reason**:

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

---

**Fix**:

- Installed and used Ubuntu 22.04
- This was easy to do in `wsl` as follows:
	- `wsl --unregister ubuntu` (removes root file structure and all user installations)
	- `wsl --install ubuntu-22.05` ("Jammy" version of Ubuntu, which supports ROS Humble)

# RViz unable to run due to display issues with Qt
2025-04-29

- RViz has dependencies on Qt for GUI functionality
- Installing RViz for a ROS distribution should install all necessary dependencies
- **NOTE**: *ROS Humble Desktop Install* bundles RViz as well
- However, improper Qt installations could make the above skip certain dependencies
- This could lead to broken Qt packages that RViz is unable to use to display

---

Here is part of the error message:

```
qt.qpa.xcb: could not connect to display :1.0 qt.qpa.plugin:
Could not load the Qt platform plugin "xcb" in "" even though it was found.
```

---

**Fix**:

- Clean all installations and dependencies of Qt and RViz
- Reinstall RViz

Personally, since I am using WSL, I did the following:

- `wsl --unregister ubuntu` (removes root file structure and all user installations)
- `wsl --install ubuntu-22.05` ("Jammy" version of Ubuntu, which supports ROS Humble)

# RViz unresponsive to mouse clicks
2025-04-29

- RViz has dependencies on Qt for GUI functionality
- Installing RViz for a ROS distribution should install all necessary dependencies
- **NOTE**: *ROS Humble Desktop Install* bundles RViz as well
- However, improper Qt installations might have broken certain functionalities
- This could have been the cause of RViz's unresponsiveness, despite RViz running

**NOTE**: *This is not an issue of processing or lag.\**

\* *This is confirmed since*...

- RViz installed via reinstalling ROS in a fresh Ubuntu root had no such issues
- Restarting the computer and killing extra processes had no effect

---

**Attempted fix 1**:

Remove and reinstall ROS 2 Humble Desktop Install.

RESULT: No effect.

**Attempted fix 1**:

Install Qt5 and its dependencies separately.

RESULT: See [previous issue](#rviz-unable-to-run-due-to-display-issues-with-qt).

---

**Fix**:

- Clean all installations and dependencies of Qt and RViz
- Reinstall RViz

Personally, since I am using WSL, I did the following:

- `wsl --unregister ubuntu` (removes root file structure and all user installations)
- `wsl --install ubuntu-22.05` ("Jammy" version of Ubuntu, which supports ROS Humble)

# Issues when trying to build ROS Humble from source
The following shell script was run to setup dependencies to build ROS Humble from source:

```sh
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

See [`ros-humble-build-from-source.sh`](./ros-humble-build-from-source.sh) for clarity on the steps.

**NOTE**: *This shell script has been updated based on the issues discussed here.*

---

Then, the following commands were run to compile ROS 2 Humble packages from source:

```
cd ~/ros2_humble
colcon build --symlink-install
```

The following errors were thrown during compilation.

## Unable to compile ROS 2 `iceoryx_hoofs` package due to missing header file
**Relevant portion of error message**:

```
--- stderr: iceoryx_hoofs
In file included from /root/ros2_humble/src/eclipse-iceoryx/iceoryx/iceoryx_hoofs/include/iceoryx_hoofs/internal/posix_wrapper/access_control.hpp:23,
                 from /root/ros2_humble/src/eclipse-iceoryx/iceoryx/iceoryx_hoofs/source/posix_wrapper/access_control.cpp:18:
/root/ros2_humble/src/eclipse-iceoryx/iceoryx/iceoryx_hoofs/platform/linux/include/iceoryx_hoofs/platform/acl.hpp:20:10: fatal error: sys/acl.h: No such file or directory
   20 | #include <sys/acl.h>
      |          ^~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/iceoryx_hoofs.dir/build.make:300: CMakeFiles/iceoryx_hoofs.dir/source/posix_wrapper/access_control.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
gmake[1]: *** [CMakeFiles/Makefile2:100: CMakeFiles/iceoryx_hoofs.dir/all] Error 2
gmake: *** [Makefile:136: all] Error 2
```

As a result, 13 packages were aborted and 321 packages were not processed.

---

**Fix**:

Installed `libacl1-dev`, which contains `sys/acl.h`, the dependency for `iceoryx_hoofs`:

```
sudo apt install libacl1-dev -y
```

Upon rerunning `colcon build --symlink-install`, `iceoryx_hoofs` was able to compile.

> **Reference**: [*Missing sys/acl.h when trying to install Humble*, **github.com/ros2/ros2/issues**](https://github.com/ros2/ros2/issues/1375)

## Unable to compile ROS 2 `tinyxml2_vendor` package due to missing dependencies
**Relevant portion of error message**:

```
--- stderr: ``
CMake Error at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
  Could NOT find TinyXML2 (missing: TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
Call Stack (most recent call first):
  /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:594 (_FPHSA_FAILURE_MESSAGE)
  cmake/Modules/FindTinyXML2.cmake:60 (find_package_handle_standard_args)
  CMakeLists.txt:7 (find_package)
```

As a result, 15 packages were aborted and 257 packages were not processed.

---

**Fix**:

Installed `libtinyxml2-dev`, which contains the necessary dependencies for TinyXML2:

```
sudo apt install libtinyxml2-dev
```



## Unable to compile ROS 2 `libcurl_vendor` package due to missing dependencies
**Initial relevant portion of error message**:

```
--- stderr: libcurl_vendor
configure: WARNING: Cannot find libraries for LDAP support: LDAP disabled
configure: error: --with-openssl was given but OpenSSL could not be detected
gmake[2]: *** [CMakeFiles/curl-7.81.0.dir/build.make:92: curl-7.81.0-prefix/src/curl-7.81.0-stamp/curl-7.81.0-configure] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/curl-7.81.0.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
```

---

**Attempted fix 1**:

Tried to install `libcurl-dev` locally.

RESULT:

```
$ sudo apt install libcurl-dev
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Package libcurl-dev is a virtual package provided by:
  libcurl4-openssl-dev 7.81.0-1ubuntu1.20
  libcurl4-nss-dev 7.81.0-1ubuntu1.20
  libcurl4-gnutls-dev 7.81.0-1ubuntu1.20
You should explicitly select one to install.

E: Package 'libcurl-dev' has no installation candidate
```

---

**Attempted fix 2**:

Selected the following package (since error message indicated requirement of OpenSSL):

```
sudo apt install libcurl4-openssl-dev
```

> **Reference**: [*Installing curl.h library \[duplicate\], **askubuntu.com**](https://askubuntu.com/questions/78183/installing-curl-h-library)

RESULT:

Same error persisted upon trying to rebuild the workspace's packages.

---

As a result, 15 packages were aborted and 259 packages were not processed.
  
### Fix 1
```
sudo apt-get install libldap2-dev
```

---


**Relevant portion of error message**:

*Upon trying to rebuild the workspace's packages*...
 
```
--- stderr: libcurl_vendor
configure: error: --with-openssl was given but OpenSSL could not be detected
gmake[2]: *** [CMakeFiles/curl-7.81.0.dir/build.make:92: curl-7.81.0-prefix/src/curl-7.81.0-stamp/curl-7.81.0-configure] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/curl-7.81.0.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
```

> **Reference**: [*Install and configure LDAP*, **documentation.ubuntu.com**](https://documentation.ubuntu.com/server/how-to/openldap/install-openldap/index.html)

### Fix 2
```
sudo apt install libssl-dev
```

This resolved all necessary dependencies for `libcurl_vendor` package.

> **Reference**: [*OpenSSL missing during ./configure. How to fix?*, **superuser.com**](https://superuser.com/questions/371901/openssl-missing-during-configure-how-to-fix)

*From the above*...

- The OpenSSL library is usually already installed
- But you have to install the header files
- Depending on your Linux distribution, you need these packages:
	- Red Hat, Fedora, CentOS - `openssl-devel`
	- Debian, Ubuntu - `libssl-dev`
	- Arch - `openssl`

### Other references
> [About `libcurl` (official site)](https://curl.se/libcurl/)

## Unable to compile ROS 2 `fastrtps` package due to missing Asio installation
**Relevant portion of error message**:

```
--- stderr: fastrtps
CMake Error at cmake/modules/FindAsio.cmake:22 (message):
  Not found a local version of Asio installed.
Call Stack (most recent call first):
  cmake/common/eprosima_libraries.cmake:206 (find_package)
  CMakeLists.txt:167 (eprosima_find_thirdparty)
```

---

**Fix**:

Installed Asio locally by running:

```
sudo apt install libasio-dev
```

> **Reference**: [*CMake Error while installing ROS2 packages with colcon*, **robotics.stackexchange.com**](https://robotics.stackexchange.com/questions/89879/cmake-error-while-installing-ros2-packages-with-colcon)

## Unable to compile ROS 2 `orocos_kdl_vendor` package
**Relevant portion of error message**:

```
--- stderr: orocos_kdl_vendor
Cloning into 'orocos_kdl-507de66'...
HEAD is now at 507de66 Fix CMake warning on Windows (#392)
Submodule 'python_orocos_kdl/pybind11' (https://github.com/pybind/pybind11.git) registered for path 'python_orocos_kdl/pybind11'
Cloning into '/root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/python_orocos_kdl/pybind11'...
CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
Please set them or make sure they are set and tested correctly in the CMake files:
EIGEN3_INCLUDE_DIR (ADVANCED)
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/doc
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/src
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/tests
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/models
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples
   used as include directory in directory /root/ros2_humble/build/orocos_kdl_vendor/orocos_kdl-507de66-prefix/src/orocos_kdl-507de66/orocos_kdl/examples

CMake Error in src/CMakeLists.txt:
  Found relative path while evaluating include directories of "orocos-kdl":

    "EIGEN3_INCLUDE_DIR-NOTFOUND"
	
CMake Generate step failed.  Build files cannot be regenerated correctly.
gmake[2]: *** [CMakeFiles/orocos_kdl-507de66.dir/build.make:92: orocos_kdl-507de66-prefix/src/orocos_kdl-507de66-stamp/orocos_kdl-507de66-configure] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/orocos_kdl-507de66.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
```

---

**Fix**:

- Specify that colcon must ignore `orocos_kdl_vendor` during building <br> *This is done by creating a blank file `COLCON_IGNORE` within `.../src/ros2/orocos_kdl_vendor/`* <br> **NOTE**: Replace `...` with the path to your workspace
- If `orocos_kdl_vendor` proves necessary, a separate installation will be attempted

### Related errors with packages containing `kdl` in their name
#### `kdl_parser`
**Relevant portion of error message**:

```
--- stderr: kdl_parser
CMake Error at CMakeLists.txt:6 (find_package):
  By not providing "Findorocos_kdl_vendor.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "orocos_kdl_vendor", but CMake did not find one.

  Could not find a package configuration file provided by "orocos_kdl_vendor"
  with any of the following names:

    orocos_kdl_vendorConfig.cmake
    orocos_kdl_vendor-config.cmake

  Add the installation prefix of "orocos_kdl_vendor" to CMAKE_PREFIX_PATH or
  set "orocos_kdl_vendor_DIR" to a directory containing one of the above
  files.  If "orocos_kdl_vendor" provides a separate development package or
  SDK, be sure it has been installed.
```

---

**Fix**:

- Specify that colcon must ignore `kdl_vendor` during building <br> *This is done by creating a blank file `COLCON_IGNORE` within `.../src/ros/kdl_parser/`* <br> **NOTE**: Replace `...` with the path to your workspace

#### `tf2_eigen_kdl`
**Relevant portion of error message**:

```
--- stderr: tf2_eigen_kdl
... (similar as previous error message)
```
**Fix**:

- Specify that colcon must ignore `tf2_eigen_kdl` during building <br> *This is done by creating a blank file `COLCON_IGNORE` within `.../src/ros2/geometry2/tf2_eigen_kdl`* <br> **NOTE**: Replace `...` with the path to your workspace

#### `tf2_kdl`
**Relevant portion of error message**:

```
--- stderr: tf2_kdl
... (similar as previous error message)
```
**Fix**:

- Specify that colcon must ignore `tf2_kdl` during building <br> *This is done by creating a blank file `COLCON_IGNORE` within `.../src/ros2/geometry2/tf2_kdl`* <br> **NOTE**: Replace `...` with the path to your workspace

#### Tip to resolve such errors together
Find all packages (directories) with "kdl" in their names using:

```
find . -name *kdl*
```

Add a blank `COLCON_IGNORE` file in each parent package. There are 3 such packages:

- `.../src/ros/kdl_parser`
- `.../src/ros2/geometry2/tf2_eigen_kdl`
- `.../src/ros2/geometry2/tf2_kdl`

Why is this valid?

`orocos_kdl_vendor` packages the KDL (Kinematics and Dynamics Library).

=> Packages containing `kdl` in their names indicate dependency on `orocos_kdl_vendor.`

### Related error with `tf2_geometry_msgs` and `robot_state_publisher` packages
These packages have dependencies on:

- `orocos_kdl`
- `orocos_kdl_vendor`
- `kdl_parser` (`robot_state_publisher` has this dep but not `tf2_geometry_msgs`)

These dependencies are unnecessary if we are omitting the use of KDL.

---

The easiest to remove these dependencies from the packages in question:

*Comment/delete any reference to these packages in their `CMake.txt` files.*

However:

`robot_state_publisher/src/robot_state_publisher.cpp` has dependencies on `kdl_parser`.

*It might be worth knowing how to install Orocos KDL in the system.* 

---

For reference, their relative paths are as follows:

- `.../src/ros/robot_state_publisher`
- `.../src/ros2/tf2_geometry_msgs`

**NOTE**: *Replace `...` with the path to your workspace.*

## Issues in installing Orocos KDL locally
***Related to previous section***

For technical details, see relevant section in [`build-ros2-humble-from-source`](./build-ros2-humble-from-source)

### Improper `g++` version
**Relevant portion of error message**:

```
-- Build files have been written to: /root/orocos_from_source/orocos_toolchain/log4cpp
[ 10%] Performing build step for 'log4cpp'
[  2%] Building CXX object CMakeFiles/orocos-log4cpp.dir/src/Appender.cpp.o
In file included from /root/orocos_from_source/orocos_toolchain/log4cpp/include/log4cpp/Appender.hh:21,
                 from /root/orocos_from_source/orocos_toolchain/log4cpp/src/Appender.cpp:11:
/root/orocos_from_source/orocos_toolchain/log4cpp/include/log4cpp/Priority.hh:107:9: error: ISO C++17 does not allow dynamic exception specifications
  107 |         throw(std::invalid_argument);
      |         ^~~~~
make[5]: *** [CMakeFiles/orocos-log4cpp.dir/build.make:76: CMakeFiles/orocos-log4cpp.dir/src/Appender.cpp.o] Error 1
make[4]: *** [CMakeFiles/Makefile2:143: CMakeFiles/orocos-log4cpp.dir/all] Error 2
make[3]: *** [Makefile:136: all] Error 2
make[2]: *** [CMakeFiles/log4cpp.dir/build.make:86: stamp/log4cpp-build] Error 2
make[1]: *** [CMakeFiles/Makefile2:112: CMakeFiles/log4cpp.dir/all] Error 2
make: *** [Makefile:136: all] Error 2
```

Focus especially on:

```
error: ISO C++17 does not allow dynamic exception specifications
```

---

**Cause**:

Source code is using a C++ standard before ISO C++17, which:

- Allows dynamic exception specifications
- *Hence could be C++14, C++11 or earlier\**

\* *Although dynamic exception specifications are deprecated since C++11.*

However, the current C++ GCC compiler does not support anything below C++17.

> **Reference**: > [*c++1z dynamic exception specification error*, **stackoverflow.com**](https://stackoverflow.com/questions/47284705/c1z-dynamic-exception-specification-error)

---

> **Additional reference**: [*Dynamic exception specification (until C++17)*, **cppreference.com**](https://en.cppreference.com/w/cpp/language/except_spec)

---

**Fix**:

By default, GCC 6.1 up to GCC 10 support the 2014 C++ standard (C++14)

> **Reference**: ["C++14 Support in GCC", *C++ Standards Support in GCC*, **gcc.gnu.org**](https://gcc.gnu.org/projects/cxx-status.html#cxx14)

**TERMINOLOGY SIDE NOTE: "GCC" in the context of C++**:

- GCC is a common shorthand term for the GNU Compiler Collection
- This name is both of the following:
	- The most general name for the compiler
	- The name used when the emphasis is on compiling C programs <br> *As the abbreviation formerly stood for "GNU C Compiler"*
- When referring to C++ compilation, it is usual to call the compiler "G++"
	- Since there is only one compiler, it is also accurate to call it "GCC" <br> *Regardless of the language in question*
	- But the term "G++" is more useful if emphasis is on compiling C++ programs

> **Reference**: [*Using the GNU Compiler Collection (GCC)*, **gcc.gnu.org**](https://gcc.gnu.org/onlinedocs/gcc-3.3.5/gcc/G_002b_002b-and-GCC.html)

Hence, I decided to downgrade G++ to G++6 (chose the lowest viable version for assurance).

**NOTE**: *Verify `g++` version running `g++ --version` in the terminal.*

Now, if you simply try to install G++6 for Ubuntu 20.04 and above, you get:

```
E: Package 'g++-6' has no installation candidate
```

Hence, I downgraded G++ to G++6 by doing the following:

**a)**<br>
Added the archive versions of the repositories bionic, main and universe to:

`/etc/apt/sources.list` (the list of repo sources to install/upgrade from)

PROCEDURE:

Open `/etc/apt/sources.list` via a text editor, e.g.: `nano`:

`sudo nano /etc/apt/sources.list`

Add `deb http://dk.archive.ubuntu.com/ubuntu/ bionic main universe` to the file.

**b)**<br>
Found and copied the key ID after the text `NO_PUBKEY` after running the following:

`sudo apt update`

Let `XXXXXXXXXX` represent this key for explanatory purposes.

**c)**<br>
Ran the following:

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys XXXXXXXXXX
sudo apt install g++-6 -y
```

**d)**<br>
Ran the following:

`sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 4`

This follows the format as follows:

`sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-[version_number] [priority]`

I chose priority 4 as it exceeds priorities of existing G++ versions.

*However, any high-enough number will do.*

**e)**<br>
Step **(d)** ensures the G++ version is 6.

We can also choose the desired default version using the following:

`sudo update-alternatives --config g++`

> **References**:
>
> - [*How to Downgrade GCC Version on Ubuntu*, **WebHostingGeeks.com**](https://webhostinggeeks.com/howto/how-to-downgrade-gcc-version-on-ubuntu/) (similar procedure for G++)
> - [`zuyu`/`ubuntu-install-gcc-6`, **gist.github.com**](https://gist.github.com/zuyu/7d5682a5c75282c596449758d21db5ed)

---

**IMPORTANT POINT 1**:

- Do not downgrade `gcc` in Ubuntu; here, `gcc` is the C compiler
- `gcc`'s version must be high enough (e.g. 11) to recognise the command option `-ffile-prefix-map=/build/ruby3.0-IyvJLJ/ruby3.0-3.0.2=` <br>For details, see ["Improper `gcc` version](#improper-gcc-version)

---

**IMPORTANT POINT 2**:

- It is advisable to return `g++` to the latest version
- This is so it can correctly compile more up-to-date C++ source files
- `g++` can be brought to a later version in the following ways:
	- De-prioritise `g++-6`: `sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 <k>` <br> *`<k>` is a placeholder for a sufficiently low priority*
	- Prioritise `g++-?` (`?` to be replaced, e.g. `g++-11)`: `sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 <k>` <br> *`<k>` is a placeholder for a sufficiently high priority*
	- Do both of the above

### Improper `gcc` version
**Relevant portion of the error message**:

```
[ 46%] Building CXX object bindings/ruby/CMakeFiles/typelib_ruby.dir/ext/typelib_ruby.o
c++: error: unrecognized command line option ‘-ffile-prefix-map=/build/ruby3.0-IyvJLJ/ruby3.0-3.0.2=.’
make[5]: *** [bindings/ruby/CMakeFiles/typelib_ruby.dir/build.make:76: bindings/ruby/CMakeFiles/typelib_ruby.dir/ext/typelib_ruby.o] Error 1
make[4]: *** [CMakeFiles/Makefile2:1008: bindings/ruby/CMakeFiles/typelib_ruby.dir/all] Error 2
make[3]: *** [Makefile:146: all] Error 2
make[2]: *** [CMakeFiles/typelib.dir/build.make:86: stamp/typelib-build] Error 2
make[1]: *** [CMakeFiles/Makefile2:217: CMakeFiles/typelib.dir/all] Error 2
make: *** [Makefile:136: all] Error 2
```

---

**Cause and fix**:

- The above was caused as I mistakenly downgraded `gcc` to `gcc-6` <br> *Procedure is same as described in ["Improper `g++` version"](#improper-g-version); just replace `g++` with `gcc`*
- The issue was fixed by upgrading `gcc` back to `gcc-11`
	- Install `gcc-11` if not already installed, using `sudo apt install gcc-11`
	- Update default version using `sudo update-alternatives --config gcc`

**NOTE 1**: *Verify `gcc` version running `gcc --version` in the terminal.*

**NOTE 2**: *In Ubuntu CLI, `gcc` is GNU C compiler while `g++` is GNU C++ compiler.*

### Missing COBRA implementation
**Relevant portion of error message**:

```
CMake Error at config/FindTAO.cmake:181 (MESSAGE):
  Could not find TAO.  Missing components: ACETAO
Call Stack (most recent call first):
  config/FindCorba.cmake:14 (find_package)
  config/check_depend.cmake:366 (find_package)
  CMakeLists.txt:106 (INCLUDE)
```

---

**Cause and fix**:

- The above was caused because `ENABLE_CORBA` was `ON` in CMake configuration
- Simply toggle this as `OFF` as follows:
	- Navigate to the cloned `orocos_toolchain` repo within your workspace
	- Run `ccmake .`; this allows you to see and modify CMake configuration
	- Type `c` to configure
	- Move to `ENABLE_COBRA` option and toggle it to `OFF`
	- Generate the configuration-based build by typing `g`

### Broken `orocos_toolchain` build
**Relevant portion of error message**:

```
-- using Ruby version 3.0.2
CMake Error at test/ruby/CMakeLists.txt:2 (file):
  file COPY cannot find
  "/root/orocos_from_source/orocos_toolchain/typelib/test/ruby/runner.in": No
  such file or directory.


CMake Error at test/ruby/CMakeLists.txt:5 (file):
  file RENAME failed to rename

    /root/orocos_from_source/orocos_toolchain/typelib/test/ruby/runner.in

  to

    /root/orocos_from_source/orocos_toolchain/typelib/test/ruby/runner

  because: No such file or directory



CMake Error at test/ruby/CMakeLists.txt:6 (file):
  file RENAME failed to rename

    /root/orocos_from_source/orocos_toolchain/typelib/test/ruby/cxx_tlbgen.in

  to

    /root/orocos_from_source/orocos_toolchain/typelib/test/ruby/cxx_tlbgen

  because: No such file or directory
```

---

**Cause and fix**:

- CAUSE: Broken build packages due to repeated failed rebuilding attempts
- FIX: Delete the cloned repo `orocos_toolchain`, reclone it and rebuild it

For details on building `orocos_toolchain` from source, check [`build-ros2-humble-from-source.md`](./build-ros2-humble-from-source.md).

## Package build failing due to memory-hogging
**Relevant portion of the error messagee**:

```
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
gmake[2]: *** [CMakeFiles/test_play_services__rmw_fastrtps_dynamic_cpp.dir/build.make:76: CMakeFiles/test_play_services__rmw_fastrtps_dynamic_cpp.dir/test/rosbag2_transport/test_play_services.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:1588: CMakeFiles/test_play_services__rmw_fastrtps_dynamic_cpp.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
gmake: *** [Makefile:146: all] Error 2
```

**NOTE**: *This error is not unique to a given package.*

---

**Cause**:

The error `killed signal terminated program cc1plus` typically indicates:

C++ compiler was forcibly terminated due to reasons such as:

- Insufficient memory (i.e. RAM)
- System-imposed resource limits

> **References**:
>
> - [*c++: fatal error: Killed signal terminated program cc1plus*, **github.com**](https://github.com/mlpack/mlpack/issues/2775)
> - [*What does this error mean? Killed signal terminated program cc1plus*, **brainly.com/question**](https://brainly.com/question/43022741)
> - [*Bug 107483 - c++: fatal error: Killed signal terminated program cc1plus due to out of memory*, **gcc.gnu.org/bugzilla**](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=107483)

In the context of colcon, it appears that:

- The build is handled in primary memory before being committed to persistent storage
- If the build size grows too large for the primary memory, we get the given error

---

**Potential fixes**:

- In some cases, it was sufficient to run the failed/aborted packages separately <br> *Especially useful for heavier packages*
- Close other running non-essential applications/processes (to reduce memory-hogging)
- Specify sequential as opposed to parallel compilation (to reduce memory-hogging) <br> *For `colcon build` option `--executor`, give `parallel` or `sequential`* <br> *Reference*: [*Build of ROS2 package: Out of memory*, **robotics.stackexchange.com**](https://robotics.stackexchange.com/questions/102684/build-of-ros2-package-out-of-memory)
- If necessary, you can increase the swap size (applicable for Linux) <br> *References for swap memory in Linux*:
	- [*How To Add Swap Space on Ubuntu 22.04* by Alex Garnett, **www.digitalocean.com**](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04)
	- [*Swap Space in Linux: What It Is & How It Works*, **phoenixNAP.com**](https://phoenixnap.com/kb/swap-space)
	- [*How to Check Swap Space in Linux*, **GeeksForGeeks.org**](https://www.geeksforgeeks.org/how-to-check-swap-space-in-linux/)
	
### Solving issues in building `rviz_default_plugins` by increasing swap space
**Importance of `rviz_default_plugins`**:

`rviz_default_plugins` is a package that, while not strictly necessary for RViz to run, greatly facilitates the integration of RViz's GUI to its visualisation functionality. RViz plugins add functionality to it, but default plugins can be thought of as core functionality without which RViz has no practical value. Hence, building this package was an important step in building ROS 2 from source.

---

**Build-related issues**:

Using `free -h`, I was able to periodically inspect memory usage as follows:

- Memory (total, used and free)
- Swap (total, used and free)

Initially, I had:

- Memory: total = 7.5Gi
- Swap: total = 2.5Gi

However, `rviz_default_plugins`'s build can easily exceed these limits.

As a result, the following was shown across the build process:

```
Starting >>> rviz_default_plugins
--- stderr: rviz_default_plugins
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
gmake[2]: *** [CMakeFiles/rviz_default_plugins.dir/build.make:1774: CMakeFiles/rviz_default_plugins.dir/src/rviz_default_plugins/displays/pose_covariance/pose_with_cov_selection_handler.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
gmake[2]: *** [CMakeFiles/rviz_default_plugins.dir/build.make:1802: CMakeFiles/rviz_default_plugins.dir/src/rviz_default_plugins/displays/relative_humidity/relative_humidity_display.cpp.o] Error 1
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
gmake[2]: *** [CMakeFiles/rviz_default_plugins.dir/build.make:1228: CMakeFiles/rviz_default_plugins.dir/src/rviz_default_plugins/displays/map/map_display.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:298: CMakeFiles/rviz_default_plugins.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< rviz_default_plugins [3min 33s, exited with code 2]

Summary: 0 packages finished [3min 39s]
  1 package failed: rviz_default_plugins
  1 package had stderr output: rviz_default_plugins
```

---

**Fix**:

**NOTE 1**: *View existing swap files/partitions using `swapon --show`.*

**NOTE 2**: *View available hard disk space using `df -h` ("df"="disk free").*

I temporarily increased available swap memory as follows:

```sh
# fallocate lets users directly manipulate allocated disk space for given file:
sudo fallocate -l 15G /swapfile # "swapfile" is just a name; any valid name is okay
# NOTE: The -l option indicates the allocation length, which is 15Gi in this case

# We can verify that the correct amount of space was reserved by typing:
ls -lh /swapfile

# EXTRA: We can make the swap file accessible only to the root's processes:
sudo chmod 600 /swapfile
# NOTE: This prevents non-root processes from using the swap file

# We can verify the permissions change:
ls -lh /swapfile

# We can now mark the file as swap space:
sudo mkswap /swapfile

# We can enable the swap file, allowing our system to start using it:
sudo swapon /swapfile

# We can verify that the swap is available:
sudo swapon --show

We can check output of the "free" utility again to corroborate the above:
free -h
```

**NOTE**: *The above on its own does not save the swap file for a new terminal session.*

In the same terminal session, I reran:

```
colcon build --symlink-install rviz_default_packages
```

**NOTE**: *The above is run from the local ROS 2 workspace.*

> **Reference**: [*How To Add Swap Space on Ubuntu 22.04* by Alex Garnett, **www.digitalocean.com**](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04)

---

**EXTRA INFO**: Peak memory + swap space usage by this build process was observed to be ~15Gi.
	
## C++ compilation issues for ROS 2 source files
Issue:

- C++ compilation errors when building using valid ROS 2 C++ sources files
- No other kind of error; purely a C++ compilation issue

---

**Fix**:

Ensure the compiler is up-to-date. How?

- Check `g++` version using `g++ --version`
- If lower (say `6.4.0`) than the latest (say `11.4.0`), do the following:
	- Ensure installations of higher/up-to-date versions of `g++` are obtained <br> E.g.: `sudo apt install g++-11`
	- De-prioritise `g++-6`: `sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 <k>` <br> *`<k>` is a placeholder for a sufficiently low priority*
	- Prioritise `g++-?` (`?` to be replaced, e.g. `g++-11)`: `sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 <k>` <br> *`<k>` is a placeholder for a sufficiently high priority*
	- Do both of the above

## RViz unable to load default plugins despite them being built
**Relevant portion of error message**:

```
[ERROR] [1746058238.954459938] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Orbit' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Orbit with base class type rviz_common::ViewController does not exist. Declared types are
[ERROR] [1746058254.625782381] [rviz2]: PluginlibFactory: The plugin for class 'rviz_default_plugins/Grid' failed to load. Error: According to the loaded plugin descriptions the class rviz_default_plugins/Grid with base class type rviz_common::Display does not exist. Declared types are
[ERROR] [1746058254.633424449] [rviz2]: ...
```

---

**Fix**:

I am not aware of the exact cause. However, what worked for me was rebuilding:

`rviz_common`, `rviz_default_packages` and `rviz2` (in this order).

If it helps, delete the existing installations and builds of these before rebuilding\.

\* *IDEA: Avoid left-over issues not covered by overriding; probably not necessary.*
	
## Unresolved error
```
E: The repository 'http://packages.ros.org/ros/ubuntu jammy Release' does not have a Release file.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
```
