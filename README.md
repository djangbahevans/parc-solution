# Team Asimovs: PARC Engineers League  

## Introduction

The 2021 PARC Engineers League challenges the various teams to build a delivery robot that can pick up a package, and deliver it to a destination autonomously, while obeying traffic rules and avoid collision with nearby objects. Deliverty robots are seeing more use world wide, especially during the coronavirus pandemic, where they could deliver products autonomously. Delivery robots could be useful in rural cocoa farming communities, where they can autonomously deliver plucked cocoa pods to the processing points, removing the need to use people, mostly children, for this purposes.

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

The navigation stack is used to run the robot to the unmapped cluttered space.

To run the solution, run the following command (goal_x, goal_y and goal_theta are the coordinate goal, assumed to be -2.0 and 10.9 respectively if not provided): <br>
` roslaunch parc-solution task3_solution.launch goal_x:=<goal_x> goal_y:=<goal_y> goal_theta:=<goal_theta> `

## Challenges Faced

* Goal sphere intefering with sensors readings, causing robot to misbehave

* Little background in ROS and computer vision meant the team had a lot of ground to cover
