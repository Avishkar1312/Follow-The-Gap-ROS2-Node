# Follow-The-Gap Algorithm &#x1F697;

The Follow the Gap algorithm is a reactive obstacle avoidance strategy that uses LiDAR data to guide autonomous vehicles through the safest and most open path. It finds the safest path by scanning for the largest gaps and moving through them. 
It is mainly used when safety is of the utmost priority



<img src="https://ars.els-cdn.com/content/image/1-s2.0-S0921889012000838-gr3.jpg" alt="FTG Demo" />       


## About this repository 
This is a ROS2 package containing a Python-based code for the follow-the-gap algorithm. The code file ("ftg.py") is placed in the directory named ftg_code. It can be used to test some sets of lidar data and generate the corresponding
steering angle, steering velocity and linear velocity

### Dependencies
1. Ubuntu 20.04
2. ROS2 Foxy- Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.

To copy the repository on your local system:

```bash

cd workspace/src
git clone https://github.com/Avishkar1312/Follow-The-Gap-ROS2-Node.git

```

## Simulation Information

This algorithm can be tested on the following open-source simulators:

1. [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros)  
2. [F1TENTH Autodrive Simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing)

After setting up the simulator 

## Working of the code
1. The lidar data is published from the simulator to the ROS2 topic "/scan". This includes the distances of surrounding points from the lidar.
2. The node "LIdar_Processing_Node" subscribes to the topic "/scan" and works on preprocessing the data and generating an array of tuples containing (angle, distance) of each point
3. The largest gap is identified and the heading angle is calculated keeping in mind the [non-holonomic constraints](https://mecharithm.com/learning/lesson/holonomic-nonholonomic-constraints-robots-103) of the bot
4. The steering angle, steering velocity and linear velocity are published to the Ackermaan Drive topic named "/drive". If simulation is not to be done, then these values can be simply printed too.
