#! /bin/sh

ESC=$(printf '\033')  # For add color on terminal output.
echo "${ESC}[1;6;36mThis script is for installing ROS 2 Humble and related modules.${ESC}[m"

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Setup Sources
sudo apt install software-properties-common -y
echo "${ESC}[1;6;36mPress Enter to continue.${ESC}[m"
# Update the apt repository key for ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
echo "Install ROS 2 packages"
sudo apt update
sudo apt upgrade -y

sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

sudo apt install ros-$ROS_DISTRO-xacro -y

# Install Gazebo and rqt plugin
echo "Install Gazebo and rqt plugin"
sudo apt -y install gazebo
sudo apt install ros-humble-gazebo-* -y
sudo apt install ros-humble-rqt-* -y

sudo apt install ros-$ROS_DISTRO-gazebo-msgs -y

# ros2_control
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers -y

# # Install (Ignition) Gazebo Fortress and rqt plugin
# echo "Install Gazebo Fortress"
# sudo apt update -y
# sudo apt install lsb-release wget gnupg -y
# sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update -y
# sudo apt-get install ignition-fortress -y
# sudo apt-get install ros-humble-ros-ign-bridge -y
# sudo apt install ros-humble-simulation -y

# Install joint state publisher and its gui tool
echo "Install joint state publisher and its gui tool"
sudo apt install ros-humble-joint-state-publisher*

# Install TF2 related
echo "Install TF2 related"
sudo apt install ros-humble-tf2* -y

# Install colcon related
echo "Install colcon related"
sudo apt install python3-colcon-clean -y

# Install PS4 controller related
echo "Install PS4 controller related"
sudo apt install joystick -y
sudo apt install jstest-gtk -y  # GUI debug tool
sudo apt install ros-humble-joy* -y  # ROS tool

# Install PlotJuggler (plugin for plotting data)
echo "Install PlotJuggler"
sudo apt install ros-humble-plotjuggler-ros -y

# Update .bashrc
echo "Update .bashrc"

echo -e "\n# ROS 2 Humble" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/lbr_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1  # When you use ROS 2 only on your PC." >> ~/.bashrc
echo "export ROS_DOMAIN_ID=16 && export ROS_LOCALHOST_ONLY=0  # When you use multiple machines via SSH connection" >> ~/.bashrc
echo "echo ROS_DOMAIN_ID = \$ROS_DOMAIN_ID && echo ROS_LOCALHOST_ONLY = \$ROS_LOCALHOST_ONLY" >> ~/.bashrc

source ~/.bashrc
