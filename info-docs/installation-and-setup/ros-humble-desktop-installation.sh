# REFERENCE: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

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
echo "INFO: Installing ROS Humble packages and binaries..."

# Updating caches for 'apt' repositories after setting up the repositories:
sudo apt update
# Ensuring the system is up to date before installing new packages:
# NOTE: This is useful as ROS 2 packages are built on frequently updated Ubuntu systems
sudo apt upgrade
# Desktop install (recommended, as it contains ROS, RViz, demos and tutorials):
sudo apt install ros-humble-desktop


echo "================================================"
echo "INFO: Sourcing installation to setup ROS in the current environment..."

source /opt/ros/humble/setup.bash
echo "NOTE: Sourcing is done using the command:"
echo "source /opt/ros/humble/setup.bash"
echo "To ensure the above is sourced for every terminal session, add the above to ~/.bashrc"
