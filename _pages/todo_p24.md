# Part 2 - Swarm robotics

**Quick Access:** [The foot-bot](https://dgarzonramos.github.io/robotics/fbot), [Lua](https://dgarzonramos.github.io/robotics/lua), [ARGoS](https://dgarzonramos.github.io/robotics/argos).

## Practical session 2.4 - Consensus achievement

Obstacle avoidance is the ability of robotic systems to move in their environment while avoiding the collision with objects and with other robots. Although rather simple, this concept is particularly important in the design of robot swarms. A robot swarm very likely will not exhibit any desirable behavior if the robots are bumping each other. This effect is more evident when there is a high density of objects or robots in the environment.

### Objective

The objective of this practical session is to design the control software for a robot swarm in which the individual robots explore an enclosed environment without colliding with objects, walls or other robots. The swarm, as a whole, must explore the environment evenly---that is, it must cover all the spaces.

### Designing the Control software

The control software of the robots is executed in the form of time steps---that is, the script is executed in the simulator once for each time step. In this experiment, the time step has a length of 100ms. In other words, each of the actions defined in the Lua script will be executed 10 times per second.

It is expected that you use the control software produced in practical session [2.1 Obstacle avoidance](https://dgarzonramos.github.io/robotics/21) to allow the robots move in the environment without colliding with the walls and other robots.

Remember that the individual actions of each robot are the ones that lead the robot swarm to achieve the desired behavior.

**Controller description**

In this practical session, the robots must explore the environment while performing obstacle avoidance. Robots can detect other robots and broadcast messages by using their range-and-bearing system. The regions of interest are defined as black spots on the ground, and they can be detected by the ground sensors of the robots. The control software can be described as a set of conditions that trigger specific behaviors on the robots.

In an _individualistic strategy_, a robot explores the environment until it detects the region of interest. After entering the region, it stops moving and waits for other robots to arrive. This implementation leads to larger times to aggregate all robots in the swarm.

A second and more _collaborative strategy_ involves the cooperation among robots. If a robot detects the region of interest, it stops moving and broadcast a message that calls other robots. If a robot does not detect the region of interest, but it receives a message from a robot that did it, it turns to move in the direction of the message that it received. If the robot does not sense the region of interest or any other robot broadcasting a message, it continues exploring.

By using either strategy, eventually all robots aggregate in the region of interest. Nevertheless, the cooperation between robots allows them to aggregate faster than what the swarm could achieve if each robot tries to find the region of interest individually.

### Examples:

These are examples of a robot swarm aggregating in an environment whit one region of interest. In each example, the robots use a different aggregation strategy.

**- Individualistic behavior:** Robots find the region of interest by themselves.

![Aggregation](https://dgarzonramos.github.io/robotics/assets/img/swarm.gif)

**- Collaborative behavior:** After finding the region of interest, robots help other robots to reach it.

![Aggregation](https://dgarzonramos.github.io/robotics/assets/img/swarm.gif)

### Running the experiment

You will design the control software in two scenarios. The first scenario is has one region of interest, and the second scenario has two of them.

1 - Enter the directory that contains the materials for this practical session.
```
cd ~/swarm_robotics/aggregation/
```
2 -  Run the scenario with a single region of interest and design a script for the control software of the robots.
```
argos3 -c TODO.argos
```
3 -  Run the scenario with two regions of interest and test the script you produced before. If necessary, adjust it so that the robot swarm is able to aggregate just in one of the two regions of interest.
```
argos3 -c TODO.argos
```
4 -  Deliver the Lua script to the teacher and teaching assistant **before** midnight.

### Solutions

Solutions will be available to download after midnight.

[//]: # (Link to solution 1 in google drive)
[//]: # (Link to solution 2 in google drive)

### Additional documentation

The following reading discusses the design of probabilistic strategies in aggregation of robot swarms.

1 - [Soysal, O. and Şahin, E. (2005). Probabilistic aggregation strategies in swarm robotic systems](https://doi.org/10.1109/SIS.2005.1501639).
