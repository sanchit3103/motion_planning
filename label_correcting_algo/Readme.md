# Autonomous Navigation using Label Correcting Algorithm 

<p align="justify">
This project focuses on the methodology to design and implement autonomous navigation of a robot in a door and key environment using one of the Dynamic Programming algorithms. The Label Correcting algorithm in forward time has been implemented in this project to determine the agent’s shortest path from start location to goal location in an 8x8 minigrid – door and key environment. The tasks include picking up the key and unlocking the door while traversing to the goal. In order to design the path planning function, the problem is formulated as a Markov Decision Process (MDP) with implementation of Label Correcting algorithm in forward time to determine the optimal control policy and value function. The results of the shortest path taken by the robot for all the known and unknown map environments are presented in the report.
</p>

## Project Report
[Sanchit Gupta, 'Autonomous Navigation in a Door & Key Environment using Dynamic Programming', ECE 276B, Course Project, UCSD](https://github.com/sanchit3103/motion_planning/blob/main/label_correcting_algo/Report.pdf)

## Project implementation in form of GIF files for different environments
<p align="center">
  
  <img src = "https://user-images.githubusercontent.com/4907348/208604724-f22f20a8-b07f-4729-bba7-b64be0909588.gif"/>, &nbsp;&nbsp; <img src = "https://user-images.githubusercontent.com/4907348/208604798-a7bf1d2b-75d6-44b8-a9fa-27eeaf0356d8.gif"/>, &nbsp;&nbsp; <img src = "https://user-images.githubusercontent.com/4907348/208604831-a8da8a94-fd6f-4a7f-9253-a8a882f55d78.gif" /> 
  
</p>

## Details to run the code

* <b> main.py: </b> Main file which should be run to execute the project. The environment name will have to be changed here to run the code for different environments.
* <b> utils.py: </b> Contains all the utility functions required for the project.
* <b> part_a.py: </b> Implentation of label correcting algorithm for environments with single door.
* <b> part_b.py: </b> Implentation of label correcting algorithm for environments with multiple doors.
