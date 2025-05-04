# REFERENCE: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

echo "INFO: Current locale..."

locale  # Check for UTF-8


echo "================================================"
echo "Updating 'apt' repositories and installing package: locales..."

sudo apt update && sudo apt install locales


echo "================================================"
echo "INFO: Updating locale..."

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


echo "================================================"
echo "INFO: Viewing updated locale..."

locale  # Verify settings


echo "================================================"
echo "INFO: Ensuring Ubuntu Universe repository is enabled..."
# NOTE 1: This repository contains many necessary installations
# NOTE 2: ROS installations are NOT available here!

sudo apt install software-properties-common
sudo add-apt-repository universe


echo "================================================"
echo "INFO: Obtaining GPG (GNU Privacy Guard) key for ROS Ubuntu binary installations..."

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "INFO: GPG key stored at '/usr/share/keyrings/ros-archive-keyring.gpg'"


echo "================================================"
echo "INFO: Adding repository http://packages.ros.org/ros2/ubuntu to list of sources for 'apt'..."
# NOTE: This repo contains binary installations of ROS packages for Ubuntu

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


echo "================================================"
echo "INFO: Installing development tools and ROS tools..."

sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# Further packages for Ubuntu 22.04 LTS and later:
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

echo "================================================"
echo "INFO: Creating a workspace and cloning all ROS 2 Humble repos..."

mkdir -p ~/ros2_humble/src
# NOTE: The -p flag in mkdir creates nested directories but only if they don't exist
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src


echo "================================================"
echo "INFO: Installing dependencies using rosdep..."

sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"


echo "================================================"
echo "INFO: Locally installing other necessary dependencies for ROS packages..."

# NOTE: These steps were added after facing the following issue detailed in:
# issue-log.md#issues-when-trying-to-build-ros-humble-from-source

sudo apt install libacl1-dev -y # Contains sys/acl.h needed by iceoryx_hoofs
sudo apt install libtinyxml2-dev # Needed for tinyxml2_vendor
sudo apt-get install libldap2-dev # Needed for libcurl_vendor
sudo apt install libssl-dev # Needed for libcurl_vendor
sudo apt install libasio-dev # Needed for fastrtps


echo "================================================"
echo "INFO: Building the code in the workspace..."

# NOTE: If you have already installed ROS 2 another way (either via debs or binary distro)!
# 1.
# Make sure that you run the below commands in a fresh environment.
# This environment must not have those other installations sourced.
# 2.
# Also ensure that you do not have source /opt/ros/${ROS_DISTRO}/setup.bash in your .bashrc.
# 3.
# You can make sure that ROS 2 is not sourced with the command printenv | grep -i ROS.
# The output of the above should be empty.

# NOTE: The following build commands are commented to allow the user to run them on their own

# Navigating to the workspace:
echo "Navigating the the workspace..."
cd ~/ros2_humble/
# Using colcon to compile the source code in ./src/ to create ROS installations:
# colcon build --symlink-install
# NOTE: colcon ("Collective Construction") is a meta-build tool

echo "------------------------------------"
echo "RUN THE FOLLOWING AT YOUR OWN DISCRETION:"
echo "colcon build --symlink-install"
echo "NOTE: Ensure you have navigated to your workspace before using this command"

echo "------------------------------------"
echo "NOTE: If you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use COLCON_IGNORE in the same manner as CATKIN_IGNORE to ignore the subtree or remove the folder from the workspace. E.g.: To avoid installing the large OpenCV library, simply run touch COLCON_IGNORE in the cam2image demo directory to leave it out of the build process."


echo "================================================"
echo "INFO: Sourcing the compiled installation..."

echo "RUN THE FOLLOWING AFTER BUILDING ROS PACKAGES IN THIS WORKSPACE:"
echo "source ~/ros2_humble/install/local_setup.bash"

echo "------------------------------------"
echo "NOTE: To ensure the above is sourced for every terminal session, add the above to ~/.bashrc"