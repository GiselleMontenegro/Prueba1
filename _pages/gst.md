---
permalink: /gst/
title: "Getting started"
author_profile: false
sidebar:
  nav: "contents"
---

---

In this section you will find materials and first instructions on how to prepare your computer to run the practical lessons of the robotics course. By the first session of the course, it is expected that you have followed all the instructions in this section and already have a computer that is ready to work.

If you have difficulties, the teacher and the teaching assistant will be happy to assist you---their contact information is available in the section [Contact](contact.md).

**Note:** in this section, the command lines to be introduced in the terminal of Ubuntu are presented in blocks, however, you must introduce them one by one. **DO NOT** copy and paste the whole block.

---

### Installing the OS: Ubuntu 18.04.4 LTS (Bionic Beaver) 64bit

To develop this course, you will need a **fresh installation** of Ubuntu 18.04 in your computer. Unfortunately, some simulations might be computationally heavy and **cannot** be run on Virtual Machines.

It is recommended that you use a fresh installation to avoid conflicts between the software that you might have pre-installed in your computer and the software that you will use in the practical sessions. For example, it is known that environments such as `miniconda` and `anaconda` might cause those issues.

The following tutorials will guide you step by step to install Ubuntu on your computer.

1 - [Download the desktop image file for Ubuntu 18.04.4 LTS 64bit](http://releases.ubuntu.com/18.04.4/).

2 - [Create a bootable USB stick(on Windows)](https://ubuntu.com/tutorials/tutorial-create-a-usb-stick-on-windows#1-overview).

3 - [Install Ubuntu desktop](https://ubuntu.com/tutorials/tutorial-install-ubuntu-desktop#1-overview).

**If you have a NVIDIA grahics card**

If you have an NVIDIA graphics card, it might be necessary to install additional drivers. To do this process, please install the `nvidia-prime` package
```
sudo apt install nvidia-prime
```
Open the additional drivers window with the commmand
```
software-properties-gtk --open-tab=4
```
Select the driver provided by NVIDIA for your specific graphics card, the `(proprietary, tested)` version; and then apply the changes

![NVIDIA](https://dgarzonramos.github.io/robotics101/assets/images/nvidia.png)

Reboot your computer and open a terminal. The following command should display the settings of your graphics card, where you can verify that it is operating normally. If so, quit the window without modifying anything.
```
nvidia-settings
```

---

### A crash tutorial to Linux Systems

The second step to get ready for the course is to follow the tutorial [The Linux command line for beginners](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview). It will introduce you the basic usage of the _terminal_ in Linux, and how to navigate and work using the _command line_.

If you have previous experience with Unix systems, you can skip this step.

---

### Installing an useful terminal manager

During the first part of the course, you might need to use many instances of the _terminal_  at the same time. In particular, you will use them to initialize, monitoring and stopping the simulations and the software you will produce. Therefore, it can be convenient to install a manager that can facilitate the managing of a large number of terminals at the same time. I suggest `terminator`, and you can install it by opening a terminal and entering the command below.
```
sudo apt install terminator
```
You can find documentation and a short introduction to the manager [here](https://terminator-gtk3.readthedocs.io/en/latest/gettingstarted.html).

---

### Downloading the course materials

Open a terminal and enter the following commands to download the course materials. The course materials will be placed in your `home` directory.
```
cd
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1uTSt13zTVY7MMs07vmxDPR68R2YHizz8' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1uTSt13zTVY7MMs07vmxDPR68R2YHizz8" -O course_materials.tar.gz && rm -rf /tmp/cookies.txt
```
You can know that the download process has finished when the terminal shows you an output similar to the one below.
```
2020-04-21 15:03:30 (4.98 MB/s) - ‘course_materials.tar.gz’ saved [43602664]
```
 Once the download finishes, decompress the file using the command
```
tar -zxvf course_materials.tar.gz --strip-components=1
```
The following directories must be available now in your $HOME directory.
```
swarm_robotics
catkin_ws
```
<!--
Install the `tree` package
```
sudo apt install tree
```
Run `tree` and verify that you have the materials of the course in the right place. To do so, compare with the image below.
```
tree TODO: List all files and directories of the course
```
![Materials tree](https://dgarzonramos.github.io/robotics/assets/img/nvidia.png)
-->
---

### Preparing the simulators - Part 1: Mobile robotics

You will use the Robot Operating System (ROS) and the Gazebo simulator in the first part of the course. ROS and Gazebo will be installed simultaneously and both are available in the repositories of Ubuntu. During this course, you will use ROS Melodic and Gazebo 9.0. Below, you are given with steb-by-step instructions for installing and configuring the files required for the practical sessions.

Open a terminal and enter the following commands in the command line to enable the ROS repositories
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Enter the following commands to start the installation
```
sudo apt update
sudo apt install ros-melodic-desktop-full
```
After the installation finishes, setup your environment with the following commands
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Install dependencies for building software packages
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```
Install dedendencies to run the simulation of the robot Summit-XL HL
```
sudo apt install ros-melodic-robot-localization
sudo apt install ros-melodic-mavros-msgs
sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-map-server
sudo apt install ros-melodic-amcl
sudo apt install ros-melodic-velocity-controllers
sudo apt install ros-melodic-twist-mux
sudo apt install ros-melodic-ros-control
```
Configure your ROS workspace
```
cd ~/catkin_ws/
catkin_make
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
You can verify that your workspace is successfully installed. To do so, enter the command
```
echo $ROS_PACKAGE_PATH
```
The workspace was configured correctly if it prints the following information
```
/home/yourname/catkin_ws/src:/opt/ros/melodic/share
```
Finally, test that you are able to run the simulator by entering the command
```
roslaunch sandbox_summit_xl sandbox.launch
```
The simulator might take several minutes to start the first time. If the interface displays correctly, you can close it by pressing `Ctrl + C` in the terminal until all processes stop.

![Gazebo interface](https://dgarzonramos.github.io/robotics101/assets/images/gzinterface.png)

**Troubleshooting**

If the terminal prints the following error press `Ctrl + C` until all processes stop.

![Gazebo error](https://dgarzonramos.github.io/robotics101/assets/images/gzerror.png)

First verify that the file `~/.ignition/fuel/config.yaml` exists. You can verify it by typing the following command
```
cat ~/.ignition/fuel/config.yaml
```
The terminal should print the following information
```
---
# The list of servers.
servers:
  -
    name: osrf
    url: https://api.ignitionfuel.org

  # -
    # name: another_server
    # url: https://myserver

# Where are the assets stored in disk.
# cache:
#   path: /tmp/ignition/fuel
```
If you get the output above, type the following command in the terminal
```
sed -i 's/ignitionfuel/ignitionrobotics/g' ~/.ignition/fuel/config.yaml
```
Now you can launch again the simulation with the following command. The simulation can again take a few minutes before starting.
```
roslaunch sandbox_summit_xl sandbox.launch
```
If the interface displays correctly, you can close it by pressing `Ctrl + C` in the terminal until all processes stop.

---

### Preparing the simulators - Part 2: Swarm robotics

You will use the ARGoS simulator in the second part of the course. To install the simulator on its current version (3.0.0-beta56), open a terminal and enter the following commands in the command line
```
sudo apt update
sudo apt upgrade
sudo apt install build-essential
sudo apt install cmake libfreeimage-dev libfreeimageplus-dev qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3 doxygen graphviz graphviz-dev asciidoc
cd ~/swarm_robotics/argos_installer
sudo apt install ./argos3_simulator-3.0.0-x86_64-beta56.deb
```
After the installation process finishes, verify that you are able to run an experiment by entering the following commands in the command line.
```
cd ~/swarm_robotics/sandbox/
argos3 -c sandbox.argos
```
If the installation was successful, the ARGoS interface will spawn. If the interface displays correctly, you can close it by pressing the icon in the top right corner of the visualization window of the interface.

![ARGoS interface](https://dgarzonramos.github.io/robotics101/assets/images/argosinterface.png)
