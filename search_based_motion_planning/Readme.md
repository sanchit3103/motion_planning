# Search based Motion Planning Algorithm to intercept a moving target

<p align="justify">
This project focuses on the methodology to design and implement a search-based motion planning algorithm for the robot to catch a moving target in any environment comprised of obstacles. The A* algorithm in forward time has been implemented to determine optimal path while planning on the fly to catch the target in shortest path possible. The environment size varies from a grid of 6 x 4 to a grid of 5000 x 5000. The target position changes dynamically and the robot is required to catch the target following an optimal path while avoiding obstacles. The results of optimal path taken by the robot in all given environments are presented in the report.
</p>

## Project Report
[Sanchit Gupta, 'Search based Motion Planning Algorithm to intercept a moving target', ECE 276B, Course Project, UCSD](https://github.com/sanchit3103/motion_planning/blob/main/search_based_motion_planning/Report.pdf)

## Project implementation in form of GIF files for different environments

<p align="center">
  
  <img src = "https://user-images.githubusercontent.com/4907348/209067511-abff2ed7-992b-4170-a6a0-580df68b3d0f.gif" height="300"/>
  <img src = "https://user-images.githubusercontent.com/4907348/209067580-66ad33a9-b5b7-4962-9ba9-57a34333c326.gif" height="300"/>
  <img src = "https://user-images.githubusercontent.com/4907348/209067853-0fb3fbaa-7a15-473f-a89c-59c0e8164318.gif" height="300"/> 
  
</p>

## Details to run the code:

* <b> main.py: </b> Main file which should be run to execute the project. The environment name will have to be changed here to run the code for different environments.
* <b> robotplanner.py: </b> Implentation of A* algorithm to plan the optimal path for the robot.
* <b> targetplanner.py: </b> Execution of target movement.
