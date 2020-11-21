---
permalink: /mrproject/
title: "Project: Cooperative box-pushing"
author_profile: false
sidebar:
  nav: "contentsmr"
---

---

The box-pushing problem has a simple definition: a box must be pushed from one place to another. It is a common benchmark on which assess the cooperation capabilities of multi-robot systems. Indeed, robots must coordinate to achieve an efficient movement of the box, so that the task can be performed efficiently.

From the mobile robotics perspective, the box-pushing problem rises many challenges related to the kinematic and dynamic behavior of the robots. On the one hand, one needs to plan the movement of the robots and how the are meant to push the box. On the other hand, planning for such scenario is difficult since the movement of the box might change drastically if the robots are not well coordinated.

In this project, we consider a box-pushing task in a cluttered environment. A team of two mobile robots must push an elongated box to a designated place, while avoiding and/or removing obstacles in the path. The robots can act cooperatively or individually, and they can be teleoperated or act autonomously.

In this practical session, you are asked to conceive an strategy to achieve the task, and whether necessary, develop and provide control software for team of robots.

---

### Problem definition

Two Summit-XL robots operate in a rectangular-shaped foot-bal field that includes the box that must be pushed, two extra movable obstacle boxes, two non-movable obstacle cylinders, and a set of cones that indicate the arrival place for the box---see figure below.

![Cooperative box-pushing](https://dgarzonramos.github.io/robotics101/assets/images/boxp.png)

**Goal**

The goal of the team of robots is to push the box to be inline with the set of cones. The performance of the team is measured by time they require to place the box inline with the cones.

**General remarks**

You are free to design the strategy that works the best for you. However, it must highlight the concepts you learned during the past practical sessions.

In this sense, you can conceive strategies that involve full-human teleoperation, human-robot collaboration and robot-robot cooperation. For any strategy you might choose, you will be asked to justify your design choices and elaborate on the advantages and drawbacks of the solution.

According to your strategy, you are allowed to ask how to get information about the position of the objects in the environment, as they are also provided by the simulator. Besides, you can also ask how to establish communication channels between the robots, if needed.

You can enter the directory of this practical session by entering the command
```
cd ~/catkin_ws/src/box_pushing/scripts
```
There you will find the following files
```
interface_node.py
interface_node_b.py
path_planning_node.py
path_planning_node_b.py
teleoperation_node.py
teleoperation_node_b.py
```
As you might notice, there are two copies for each file, each of which corresponds to a script you can use to control one of the robots. A first robot is labeled as `robot`, and the second robot is labeled as `robot_b`. The two robots are identical and differ only in the name used characterize them.

**IMPORTANT:**

1 - Set the properties of the files for being executable.
```
chmod +x interface_node.py
chmod +x interface_node_b.py
chmod +x path_planning_node.py
chmod +x path_planning_node_b.py
chmod +x teleoperation_node.py
chmod +x teleoperation_node_b.py
```

2 - Add the following lines into the scripts `path_planning.py` and `path_planning_b.py`. They must be placed with the rest of pre-defined global variables.
```
# Pre-defined global variables

_model_index = 0       # New line to be added
_model_found = False   # New line to be added
_truth_pose = Pose2D()
_cmd_vel = Twist()
```
3 - In the scripts `path_planning.py` and `path_planning_b.py`, replace the following line in the `while` loop of the `main` function
```
# pub_odom.publish(_cmd_vel)  # Line must be removed
pub_vel.publish(_cmd_vel)     # Line must be added
```
By mistake, they are missing in the script that you downloaded.

---

### Experiment

The goal of this project is to conceive, implement and evaluate a control strategy for a team of two robots operating in the same workspace. It is expected that you use the teleoperation interface, or the path-planning control software you developed in previous sessions.

**Launching the experiment**

1 - Open a terminal and enter the directory that contains the materials for this practical session.
```
cd ~/catkin_ws/src/box_pushing/scripts
```
2 - Run the simulation in Gazebo.
```
roslaunch box_pushing box_pushing.launch
```

**Experimental setup**

At the beginning of the experiment, two Summit-XL are spawned the scenario. The robots, boxes, and cylinders do not change their position between experiments.

**Testing your control software**

You can test your implementation by running the appropriate scripts according your solution. They might defer in regard of your strategy: full teleoperation, human-robot collaboration, robot-robot cooperation. You can run them by using one of these commands
```
rosrun box_pushing interface_node.py
rosrun box_pushing interface_node_b.py
rosrun box_pushing path_planning_node.py
rosrun box_pushing path_planning_node_b.py
rosrun box_pushing teleoperation_node.py
rosrun box_pushing teleoperation_node_b.py
```
If no error is reported, the robot will move according to the velocities you set. If you wish to try something else, click on the terminal running the teleoperation interface and press `Ctrl + C` until the process stops.

If you wish to check the velocity that you are sending to the robots, open a new terminal and enter the one of these commands
```
rostopic echo /robot/cmd_vel/
rostopic echo /robot_b/cmd_vel/
```
If you run the scripts `path_planning_node.py` or `path_planning_node_b.py` You can also get the pose of the robots with the commands
```
rostopic echo /robot/mapping/truth_pose
rostopic echo /robot_b/mapping/truth_pose
```

---

### Evaluation

The deliverable of the project is the control software that you will develop, and a video demonstrating your solution. The script must be delivered to the teacher.

The evaluation criteria are the following:

1 - Originality of the solution.

2 - Performance of the control software.

3 - Mobile robotics principles involved in the solution.

You might be asked to answer a few questions about your implementation and design choices.

---

### Further readings

The following readings show examples of multi-robot cooperation. In the first article, a team of two legged robots solves a box-pushing problem. In this research, the authors use a cooperation strategy that involves the sensing, action and control of the two robots. In the second reading, the authors show three examples of multi-robot systems that operate using ROS. In the third reading, a multi-robot system is used to safely conduct a search and rescue mission by using teleoperation.

1 - [Mataric, M. J. et al. (1995). Cooperative multi-robot box-pushing.](https://doi.org/10.1109/IROS.1995.525940)

2 - [Garzón, M. et al. (2017). Using ROS in multi-robot systems: experiences and lessons learned from real-world field tests.](https://doi.org/10.1007/978-3-319-54927-9_14)

3 - [De León, J. et al. (2016). From video games multiple cameras to multi-robot teleoperation in disaster scenarios.](https://doi.org/10.1109/ICARSC.2016.41)
