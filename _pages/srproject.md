---
permalink: /srproject/
title: "Project: Foraging with forbidden areas"
author_profile: false
sidebar:
  nav: "contentssr"
---

---

The activity of food search and retrieval is commonly referred to as _foraging_. In swarm robotics, foraging is a commonly used task to compare different algorithms for exploration (what is the best way to discover interesting places in the environment?), division of labor (who should explore? for how long?), etc.

In the most general setting, food items are scattered in an environment at locations unknown to the robots and the robots need to explore the environment, find the food, and take it to the nest. The foraging environments often contain cues, such as light sources, that help the robots in navigating through the environment.

In this project, we consider a foraging environment with a forbidden area: robots than enter this area automatically lose the item that they carry, if any. The swarm is distributed in the arena and the location of the food source is originally unknown to the robots. The swarm can act cooperatively to discover and keep track of the location of the food source for an efficient foraging, and in the same way, to avoid the forbidden area.

In this practical session, you are asked to develop and provide control software for the robot swarm, and to evaluate its performance.

---

### Problem definition

The robot swarm operates in a diamond-shaped arena that includes a food source, a nest, and one forbidden area. The food source is represented by a black circle, the nest is represented by a white area in the bottom the diamond, and the forbidden area is a gray rectangle---see figure below. A light is placed on top of the food source to indicate its position to the robots. When a robot enters the forbidden area while carrying an item, the item is automatically lost.

**Goal**

The goal of the robot swarm is to retrieve and transport items from the food source to the nest. The overall performance of the swarm is measured by the number of items it is able to collect during a fixed experiment time. Each experiment is automatically terminated after 1000 seconds (10000 time-steps). More precisely, the performance of the swarm is described by
```
Max N_d
```

with `N_d` being the number of items successfully delivered to the nest (that is, items carried by robots from the food source to the nest without entering the forbidden area).

**Swarm composition**

The swarm comprises 30 homogeneous robots. The robots are equipped with the following sensors and actuators:
```
colored_blob_omnidirectional_camera
range_and_bearing
light
motor_ground
proximity
wheels
leds
```

**General remarks**

1 - The maximal wheel velocity should not exceed 30 cm/s.

2 - ARGoS has been configured so that the robots collect items on the food source and drop them at the nest and at the forbidden area automatically. A robot can only carry one item at a time.

3 - The positions of the nest and the forbidden area is fixed, but the position of the food source can change from one experiment to another. The nest corresponds to the white area, the food source is the black spot, and the forbidden area is indicated dark shade of gray. See the figure below (top view of the arena).

![Foraging with forbidden areas](https://dgarzonramos.github.io/robotics101/assets/images/for1.png)

---

### Experiment

The goal of this project is to design, implement and evaluate control software that aims to maximize the number of items delivered at the nest. It is expected that the control software demonstrates a _cooperative_ behavior: one that takes advantage of the swarm's principles.

ARGoS is configured to automatically dump data on a file `output.txt`. This file contains a table with two columns:
```
- CLOCK: Column indicating the current step
- ITEMS: Column indicating the number of items collected so far
```

**Setting up the code**

1 - Enter the following commands in a terminal in order to prepare the code for the experiment.
```
cd ~/swarm_robotics/foraging/       
mkdir build                        
cd build                            
cmake ../src                        
make                                
```
2 - Set the environment variable `ARGOS_PLUGIN_PATH` to the path in which the `build/` directory is located.
```
export ARGOS_PLUGIN_PATH=$HOME/swarm_robotics/foraging/build/
```
3 - You can also put this line into your `$HOME/.bashrc` file, so it will be automatically executed every time you open a terminal. You add the line by entering the following command.
```
echo 'export ARGOS_PLUGIN_PATH=$HOME/swarm_robotics/foraging/build/' >> ~/.bashrc
```
4 - Run the experiment to check that everything is OK.
```
cd ~/swarm_robotics/foraging/        
argos3 -c foraging.argos             
```
If the usual ARGoS interface appears, you're ready to go.

---

### Evaluation

The deliverable of the project is the control software that you will develop. The script must be delivered to the teacher.

The evaluation criteria are the following:

1 - Originality of the solution.

2 - Performance of the control software measured over a series of experimental runs.

3 - Swarm behaviors and principles involved in the solution.

You might be asked to answer a few questions about your implementation and design choices.

---

### Further readings

The following reading shows an example of a robot swarm performing foraging. An interesting point in the experiment is that the swarm self-organizes and tackles the problem by dividing the tasks in smaller sub-problems. Then, the robots adopt different behaviors that tackle these sub-problems independently.

1 - [Ferrante, E. et al. (2015). Evolution of self-Organized task specialization in robot swarms.]( https://doi.org/10.1371/journal.pcbi.1004273)
