# Part 2 - Swarm robotics

**Quick Access:** [The foot-bot](https://dgarzonramos.github.io/robotics/fbot), [Lua](https://dgarzonramos.github.io/robotics/lua), [ARGoS](https://dgarzonramos.github.io/robotics/argos).

## Practical session 2.3 - Synchronization

Robot swarms are asynchronous systems by definition---the distributed nature of a swarm does not allow for a single clock to which all the robots have access. Hence, if robots in the swarm are meant to execute a task at the same time, they need to synchronize first. Synchronization expresses therefore the ability of robot swarms to act simultaneously without the need of a central entity that coordinates the group.

Commonly, synchronization behaviors in swarm robotics are associated to the behavior of fireflies. Although a single firefly has a reduced sensing range and only perceives a few of its peers, large groups of these insects are able to synchronize and produce light synchronously.

### Objective

The objective of this practical session is to design the control software for a robot swarm in which the individual robots must synchronize and light their LEDs at the same time.

### Designing the Control software

The control software of the robots is executed in the form of time steps---that is, the script is executed in the simulator once for each time step. In this experiment, the time step has a length of 100ms. In other words, each of the actions defined in the Lua script will be executed 10 times per second.

Remember that the individual actions of each robot are the ones that lead the robot swarm to achieve the desired behavior.

**Controller description**

In this practical session, the robots do not need to move. Robots can detect other robots and broadcast messages by using their range-and-bearing system. Robots can use their LEDs to emit color signals, and they can perceive the LEDs of their peers by using their omnidirectional camera.

The implementation of the control strategy follows the behavior of the fireflies. Each robot continuously emit a signal that must be synchronized with the signal that other the robots in the swarm are emitting.

- Initialization

Every robot must emit a signal with its range-and-bearing system and light its LEDs after a fixed number of time steps (`T`)---in other words, all robots emit the signal at the same frequency (tip: set a counter). In order to start from an asynchronous state, the first time when the signal is emitted is randomly selected by each robot in a time that lies between `0` and `T`. In this sense, the emitting phase is shifted for each robot and must be synchronized during the experiment.

- Synchronization

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
