# RobotMotionPlanning_TermProject
### This serves as the project report for the term project for the Robot Motion Planning class (ITCS 6152) under Dr. Srivinas Akella by Archit Khullar 801038938 
### Problem Statement:
* Given a parking space scenario with rectangular shaped obstacles (representing cars), consisting of parking spaces (parallel and reverse) with a rectangular car like differential drive bot, its initial position and multiple possible goal positions (which represent all the parking spaces that the bot car can be parked at). The final output of this project will be a path from its initial position to one of the final positions, which will be the closest or the fastest parking goal position.

###	Robot type and constraints:
* The project is in a planar (2D) space with a single rectangular car like robot with differential drive constraint to a DOF as R2 * S1, with motion allowed only in forward direction, hence R2 and steering angle Q. The constraints mathematically are:
  * Car length = 8 pixels
  * max_steering_angle = 1.5 radians
  
* Finally, after much research RRT was choosen for this as the previously proposed technique of Reduced Visibility graph Roadmap methodology would fail for a robot with differential drive. RRt was finally implemeted fro the pathplanning in <b>MATLAB</b>

* A map for the problem space in 2d was constructed by stitching 3 images for the boundary, parking spaces and the available parking spaces as follows

 ![Stationary](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/stationary.bmp)
 ![Road_Markings](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/road_markings.bmp)
 ![Parked Cars](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/parked_cars.bmp)
 ![Combined](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/combined.bmp)
 
 * After this, Primitive motions for the above mentioned constarints on the car were implemented, which can be found  [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/PrimitiveMotion.m), which looks like this:
 ![Primitive Motion of the Differential Drive car like robot](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/primitive.JPG)
 
 * In the next module, a normal RRT was implemented from a given initial to a given goal position which can be found [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/NormalRRT.m), whcih looks like:
 ![Normal RRT](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/normal%20RRt.JPG)
 
 * For the third module, a RRT with these primitive motion contrainst were implemented which can be found [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/RRT_NonHolonomic_R2S1.m), whcih looks like:
 ![Combined RRT](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/NH%20RRT.JPG)
  
### Conceptual Description (Nonholonomic):
* The mobile robots are known to be nonholonomic, i.e.., they are subject to nonintegrable equality nonholonomic constraints involving the velocity. In other words, the dimension of the admissible velocity space is smaller than the dimension of the configuration space. In addition, the range of possible controls is usually further constrained by inequality constraints due to mechanical stops in steering mechanism. 
	

C = R2*S1 
q (x, y, theta)
-dx*sin(theta)+dy*cos(theta) = 0

dx = v*cos(fi)*cos(theta) 
dy = v*cos(fi)*sin(theta) 
dtheta = (v/L)*sin(fi)

v=[-1, 1] 
|fi| <= MAXfi

distance (p, q) = [(p.x-q.x)^2 + (p.y-q.y)^2 + A*min((p.theta-q.theta)^2 + (p.theta-q.theta+2*PI)^2 + (p.theta-q.theta-2*PI)^2) ] ^0.5
