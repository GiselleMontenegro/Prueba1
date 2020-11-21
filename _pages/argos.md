---
permalink: /argos/
title: "ARGoS3 simulator"
author_profile: false
sidebar:
  nav: "contentssr"
---

---

[ARGoS](https://www.argos-sim.info/) is a multi-physics robot simulator developed at IRIDIA, Université Libre de Bruxelles. It can simulate large-scale swarms of robots of any kind efficiently. You will use ARGoS to work on the practical sessions of the swarm robotics part of the course.

### Getting ARGoS

The instructions about how to install ARGoS on your computer are provided in the section [Getting Started](https://dgarzonramos.github.io/robotics/gst).

### Running ARGoS

To run ARGoS, you need a file with the extension `.argos` that configures the program for a specific experiment---`.argos` is an extension based on XML. All the necessary files are provided in the materials of the course, available in the section [Getting Started](https://dgarzonramos.github.io/robotics/gst).

After installing ARGoS, you can run the simulator by opening a terminal and entering the following command
```
argos3 -c myfile.argos
```
This will open the two windows of ARGoS: a simulation window and a Lua code editor.

![ARGoS interface](https://dgarzonramos.github.io/robotics/assets/img/argos.png)

The code editor opens a template of a Lua script with a few empty functions where you will develop your controller; each function has comments that explain what it does. In this course you will work only with homogeneous robot swarms---that is, all robots in the swarm are identical and independently execute the same control software. The control software that you develop in the editor is ported to all robots in the swarm.

To execute your script on the robots, follow the numbered steps in the image above. First, click on the `Save the current file` button to store the script in a file with the extension `.lua`. Then, upload the control software to the robots by clicking on the `Execute code` button. Finally, start running the experiment by clicking on the `Play experiment` button.

Note that the template script is empty by default. It is therefore normal that the robots do not do anything when the script is executed as it is.

If you wish to open a previously saved script, click on the `Open a file` button. The interface will only look for files with the extension `.lua`. Therefore, make sure that you store your controllers with that extension.

### Visual aids to run the experiments

On the left side of the simulation window there are predefined camera positions to visualize the behavior of the robots. However, you are free to move the environment. Often, the best way to visualize the behavior of the robots is using the top-view perspective.

ARGoS also allows you to display extra information on the simulation window, such as the abstract `rays` that indicate the ranges of perception of some sensors. In particular, you can enable this ray-tracing for the proximity sensors, light sensors, range-and-bearing system, and omnidirectional camera. You can enable the visualization of the rays by modifying the declaration of the sensors in the `.argos` files. For example, set show_rays to true/false to enable/disable the visualization of the proximity sensor.
```
<footbot_proximity implementation="default" show_rays="true" />
```
