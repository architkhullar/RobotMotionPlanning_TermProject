# RobotMotionPlanning_TermProject
### This serves as the project report for the term project for the Robot Motion Planning class (ITCS 6152) under Dr. Srivinas Akella bu Archit Khullar 801038938 
### Problem Statement:
* Given a parking space scenario with rectangular shaped obstacles (representing cars), consisting of parking spaces (parallel and reverse) with a rectangular car like differential drive bot, its initial position and multiple possible goal positions (which represent all the parking spaces that the bot car can be parked at). The final output of this project will be a path from its initial position to one of the final positions, which will be the closest or the fastest parking goal position.

###	Robot type and constraints:
* The project is in a planar (2D) space with a single rectangular car like robot with differential drive constraint to a DOF as R2 * S1, with motion allowed only in forward direction, hence R2 and steering angle Q. The constraints mathematically are:
  * Car length = 8 pixels
  * max_steering_angle = 1.5 radians
  
* Finally, after much research RRT was choosen for this as the previously proposed technique of Reduced Visibility graph Roadmap methodology would fail for a robot with differential drive. RRt was finally implemeted fro the pathplanning in <b>MATLAB</b>

* A map for the problem space in 2d was constructed by stitching 3 images for the boundary, parking spaces and the available parking spaces as follows
<img src="" alt="Smiley face" height="42" width="42">
