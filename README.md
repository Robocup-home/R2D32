# Setup Instructions

# 1. Preparation

## Optional step install low latency kernel for improved real time response from the robot

```sh
sudo apt update
sudo apt upgrade -y
sudo apt-get install -y --install-recommends linux-lowlatency
sudo reboot
```
  

## 1.1 Install essential build tools

Install essential build tools. Remove brltty package which interferes with CH340 USB serial bridge on some esp32 boards.

```sh
sudo apt remove -y brltty
sudo apt install -y python3-venv build-essential cmake git curl
```


## 1.2 Install PlatformIO

### 1.2.1 Bash

```sh
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm get-platformio.py
echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
source ~/.bashrc
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

### 1.2.2 ZSH

```sh
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm get-platformio.py
echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.zshrc
source ~/.zshrc
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

## 1.3 Install ROS2 Humble


```sh
export ROS_DISTRO=humble
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-desktop ros-dev-tools python3-colcon-common-extensions python3-pip
sudo rosdep init
rosdep update
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## 1.4 Install micro-ROS

### 1.4.1 Bash

```sh
mkdir ~/uros/src -p
cd ~/uros/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
cd ~
echo "source \$HOME/uros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.4.2 ZSH

```sh
mkdir ~/uros/src -p
cd ~/uros/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
cd ~
echo "source \$HOME/uros_ws/install/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```
