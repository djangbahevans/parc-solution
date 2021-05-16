# Team Asimovs: PARC Engineers League  

## Introduction

In this section, you should give an overview of the competition task, briefly describe your understanding of delivery robotics and how relevant it is in an African context (*This should be only 4-5 sentences*)

**Team Country:** Ghana

**Team Member Names:**

* Evans Djangbah (Team Leader)
* Isaac Atia

## Dependencies

**Packages needed are:**

* `ros_control`: ROS packages including controller interfaces, controller managers, transmissions, etc.
  * `$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`

* `opencv`: Open-source computer vision library.
  * `$ sudo apt-get install python-opencv`

* `numpy`: A python package for scientific computing.
  * `$ sudo apt-get install python-numpy`

* `cv_bridge`: CvBridge is a ROS library that provides an interface between ROS and OpenCV.
  * `$ sudo apt-get install ros-melodic-cv-bridge`

* `tf`: tf is a package that lets the user keep track of multiple coordinate frames over time.
  * `$ sudo apt-get install ros-melodic-tf`
  * `$ sudo apt-get install ros-melodic-tf2`

## Task 1

A graph based path planner plans the path to use. A combination of Lane Detection with OpenCV and obstacle avoidance is used to get to the locations set by the planner.

To run the solution, run the following command (goal_x and goal_y are the coordinate goal, assumed to be -12.32 and 10.9 respectively if not provided): <br>
` roslaunch parc-solution task1_solution.launch goal_x:=<goal_x> goal_y:=<goal_y> `

## Task 2

Computer vision (OpenCV) is used to detect the traffic lights. A simple go to goal is used to drive the robot to the goal when traffic is green.

To run the solution, run the following command (goal_x and goal_y are the coordinate goal, assumed to be -2.0 and 10.9 respectively if not provided): <br>
` roslaunch parc-solution task2_solution.launch goal_x:=<goal_x> goal_y:=<goal_y> `

## Task 3

Include a brief description of your approach to the solution (*This should be only 3-5 sentences*)

Write the command required to run your solution. Should be in this format: <br>
` roslaunch parc-solution task3_solution.launch `

## Challenges Faced

* Goal sphere intefering with sensors readings, causing robot to misbehave

* Little background in ROS and computer vision meant the team had a lot of ground to cover
