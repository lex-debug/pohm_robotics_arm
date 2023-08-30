# POHM Robotics Arm
POHM stands for palm oil harvesting machine. This repository contains the algorithms of the robotics arm for harvesting the FFB such as inverse kinematics and so on.

## Table of Contents
- [Pre-requisites](#pre-requisites)
- [Environment](#environment)
- [How to install Ubuntu 20.04 in a Window OS](#how-to-install-ubuntu-2004-in-a-window-os)

### Pre-requisites
- Storage: Preferrably 20GB

## Environment
- Operating System: Ubuntu 20.04
- IDE: VSCode
- Programming Language: Python 3.8.10
- ROS Version: Foxy

## How to install Ubuntu 20.04 in a Window OS
1. Open the Microsoft Store. You might need an account for the Microsoft Store. Go ahead and register an account or use hotmail.
2. Inside the search bar, type "Ubuntu"
3. Select the Ubuntu without any version at the back and click install. This will install the latest LTS version.
![Microsoft Store Install Ubuntu](https://github.com/lex-debug/pohm_robotics_arm/blob/main/Screenshot2023-08-29154157.png)
4. After installation, you can now open the Ubuntu from your window searh box. You might need to create a credentials such as username and password for your root.
![Ubuntu on Window](https://github.com/lex-debug/pohm_robotics_arm/blob/main/Screenshot2023-08-29155022.png)

## How to install ROS Foxy
1. Copy and paste this command inside the terminal window:
```sh
locale
```
```sh
sudo apt update && sudo apt install locales
```
```sh
sudo locale-gen en_US en_US.UTF-8
```
```sh
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```
```sh
export LANG=en_US.UTF-8
```
```sh
locale
```
2. Add the ROS2 Repositories
```sh
sudo apt update && sudo apt install curl gnupg2 lsb-release
```
```sh
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```sh
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
3. Install the ROS 2 Packages
```sh
sudo apt update
```
```sh
sudo apt install ros-foxy-desktop
```
4. Set up the Environment Variables
```sh
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```


