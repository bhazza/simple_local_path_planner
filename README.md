# simple_local_path_planner

This repository contains the implementation of a simple local path planner to satisfy the requirements of the [BotsAndUs coding exercise](docs/Robotics_Engineer_Coding_Exercise_-_Google_Docs.pdf).

## Usage
<!-- Todo -->
This package can be used in place of any standard ROS local path planner package. 

## Testing
All testing has been carried out using Ubuntu 22.04 and ROS Noetic (via Docker). It may work with other versions, but this is untested.

### Turtlebot3 Simulation & Docker
<!-- Todo -->
As per the non-functional requirements, the planner must be testable in Gazebo using [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-navigation-with-turtlebot3). For convenience, a docker implementation of this is provide. 

Dependencies:
 - Docker & Docker Compose: Suggest installing [Docker Desktop](https://docs.docker.com/desktop/) which contains all docker dependencies. 

#### Build

    docker compose build

#### Run
Note: The following instructions assume docker is being run in a linux environment. Please refer to [ROS GUI Docker Tutorial](http://wiki.ros.org/docker/Tutorials/GUI) for other support.

    xhost +local:root # Warning: Unsecure. Refer to above reference tutorial for alternative methods
    docker compose run simplelocalpathplanner

    # TODO replace the following once development finished
    cd /catkin_ws
    catkin_make
    source deve/setup.bash
    roslaunch simple_local_path_planner turtlebot3_navigation.launch



### Unit Testing
<!-- Todo -->
Unit testing uses gtest


## Requirements Checklist


