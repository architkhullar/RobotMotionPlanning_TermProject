# RobotMotionPlanning_TermProject
### This serves as the project report for the term project for the Robot Motion Planning class (ITCS 6152) under Dr. Srivinas Akella by Archit Khullar 801038938 
### Problem Statement:
* Given a parking space scenario with rectangular shaped obstacles (representing cars), consisting of parking spaces (parallel and reverse) with a rectangular car like ro bot, its initial position and multiple possible goal positions (which represent all the parking spaces that the bot car can be parked at). The final output of this project will be a path from its initial position to one of the final positions, which will be the closest or the fastest parking goal position.

### Robot type and constraints:
* The project is in a planar (2D) space with a single rectangular car like robot DOF as R2 * S1, with motion allowed only in both the directions, i.r. forward and reverse. Though the car has three degrees of freedom i.e. the position(x,y) and orientation(theta). The driver has only two controls, i.e. the accelerator to control the position and the steering wheel to control the orientation of the car. Control system equations for such a system can wee written as:

!['theta; signifies the orientation, 'v' is the linear velocity and 'omega' is the steering angle](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/equations.JPG)

where 'theta; signifies the orientation, 'v' is the linear velocity and 'omega' is the steering angle
 
* With this in mind, the nonholonomic car like robot has is restricted by some kinematics and dynamics conditions. Hence, we will only extend the tree towards random configurations on trajectories allowed under these conditions.
* For this, primitive motions can be made from the equations defined above; they are:
	* ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/1.JPG)
	* ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/2.JPG)
	* ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/3.JPG)
	* ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/4.JPG)
 	* Car length = 7.5 pixels; selected in our case and can be modified
  	* max_steering_angle = 1.1 radians which is approximately 63 degrees; selected in our case and can be modified
	* dt represents one time step, whcih in our implemenation has been selected as 0.1 seconds
	
* These equations are used to generate the orimitive motions of our robot which are explained and plotted below
  
* Finally, RRT was choosen for this and this was very basic and the previously proposed technique of Reduced Visibility graph Roadmap methodology would fail for a robot with car like constraints. RRt was finally implemeted for the pathplanning in <b>MATLAB</b>

* **Also, deviating from the initial proposal, I decide to skip the C-space calculation and instead create an image map with C-Space already computed.**

* so, if we have already have the C-space computed in the map, I can treat the robot as a point robot. **Even though we have done this we will have to take into account the length of the car as that will determine the turning radius** - This has been taken care while generating the primitives

* A map for the problem space in 2d was constructed by stitching 3 images for the boundary, parking spaces and the available parking spaces as follows

 ![Stationary](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/stationary.bmp)
 ![Road_Markings](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/road_markings.bmp)
 ![Parked Cars](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/parked_cars.bmp)
 ![Combined](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/combined.bmp)
 
 ### Thought Process while starting: Coming to the algorithm implementation
 * Though I knew, the theory and the working of the RRT, I didn;t know how to start after I had made the map and folowwing were the things I was not able to decide:
 	* which tool to use? <b>Saurav</b> had suggested <b>KLAMPt</b>, I found controlling the environment cariables far too complicated.
	* I wanted to stick to basic 2d and I thought of doing it in OpenGL for C++ as I wanted to stick to C++ instead of PyOpenGL, PyGame that I initially thought.
	* I always wanted to get into MATLAB and hence I though this would be a good oppurtunity
 	* what data structure can I use?
	* how will I visualize the constant generating of nodes and searching and connecting of nodes
	* How will I add nonholonomic constraints?
	
### Environment:
* I used a simple 2-D world in MATLAB with 600 * 600 size to start, without any obstacles
* I decied to proceed with the plain algorithm first and later worry about obstacles and collision checking 

### How to build the tree?
* I used **Nested** class/struct in MATLAB, i.e. each node contains information about x,y-coordinate, index of parent node and index of the node.
* The tree will start with initial configuration and hence we know the starting x and y coordinate of the intital postion

### Building it further using random generator: I do it as follows:
**I Generating a random configuration** - Tjere are a lot of ways to do this like **uniform random sampling, Gaussian sampling, deterministic sampling**. 
* <u>In my solution, I have used common uniform sampling technique using 'rand' command which generates a random sample in uniformly distributied fashion. This sampling technique can be slow in producing final result since there is no way to direct the search towards the goal configuration</u>
* **To increase the speed of algorithm I could have added bias in the search function towards the goal configuration. This is achieved by sampling the goal configuration with some probability**

**II Finding the nearest node from the randomly generated node** - This is done by calculating distance of the random configuration from each node in the tree, Euclidean distance is used for this. All the distances are stored in an array and the minmun s=distance and its corresponding node is found. **Here collision detection should have been added**  

**II Extending the tree towards the randomly generated node** - To do this either the random configuration can be added directly to the tree as new node or the tree is extended one step in the direction of the random node. This step size choosen randomly. This can be further optimized for better results
	
### Conceptual Description (Nonholonomic):
* The mobile robots are known to be nonholonomic, i.e. they are subject to nonintegrable equality nonholonomic constraints involving the velocity. In other words, the dimension of the admissible velocity space is smaller than the dimension of the configuration space. In addition, the range of possible controls is usually further constrained by inequality constraints due to mechanical stops in steering mechanism. 

### What is RRT?
Rapidly-exploring Random Tree is a sampling based motion planning algorithm. It searches high dimensional spaces by incrementally building random tree by generating random samples. This process of building a randomly generated tree and connecting it from starting to goal configuration is as follows:

* Sample a random configuration
* Find the node in the tree which is nearest to the random configuration
* Extend the tree towards the random configuration
* Repeat steps 1 to 3. Stop iteration on reaching goal or within a threshold of the goal

* **After all this, I decided to start with basic RRT while working on the Non Holonomic primitives simultaneously wheich were like:
 
 * Primitive motions for the above mentioned constarints on the car were implemented, which can be found [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/PrimitiveMotion.m), which looks like this:
 ![Primitive Motion of the car like robot](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/primitive.JPG)
 
 * Normal RRT was implemented from a given initial to a given goal position which can be found [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/NormalRRT.m), whcih looks like:
 ![Normal RRT](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/normal%20RRt.JPG)
 ### How to genrate the RRT with the nonholonomic constraints (CHANGES HAVE BEEN DONE HERE w.r.t to taking into account the orientation as well)

* Similar to the RRt drawn before for holonomic robots, uniform sampling and biased sampling techniques are implemented. For biased sampling different probabilities can be used and checked as to how it affects the performance of the algorithm.

### Finding the Nearest node: 
* As we have three degrees of freedom x, y-coordinate and orientation θ for the given car like nnoholnomic robot. Hence while calculating the distance of random node from the nodes in the tree along with x,y-coordinate we also have to take into account θ. To do this in the Euclidean distance equation, under the square root, we add the term

```((180/pi)^2)min([(theta_rand - rrt_tree.node(i).theta)^2,(theta_rand - rrt_tree.node(i).theta - pi)^2,(theta_rand - rrt_tree.node(i).theta + pi)^2] )```

* here 'theta_rand' represents the random configuration generated orientation, s.t., 'rrt_tree.node(i).theta' is the ith node's orientationin the tree.
* The constant (180/pi)^2 is resulted from angle conversion (radians->degree) 
* First term is self explanatory.
* The 2nd and 3rd term are used to handle the direction the car is facing which can be as follows:
* in direction of random configuration
* in opposite direction of random configuration

* and the hence the distance is calculated as:

``` distance(i) = sqrt( (x_rand - rrt_tree.node(i).x)^2 + (y_rand - rrt_tree.node(i).y)^2 +  ((180/pi)^2)*min( [ (theta_rand - rrt_tree.node(i).theta)^2, (theta_rand - rrt_tree.node(i).theta - pi)^2, (theta_rand - rrt_tree.node(i).theta + pi)^2 ] )   );```

### Extension of the tree towards the random configuration:

* This being the most difficult part is handled using the pritives drawn above. Motion primitives are generated from the tree node closest to the random configuration then end point of motion primitive which is closest to the random configuration is added as the new node and the trajectory as the edge. 

* For example consider two point each represented by green and black dot such that the former represents a random configuration and later nearest node in the tree to green dot.
* For our example we consider that the robot's orientation is 0 degree at the tree node.
Next the end points of motion premitives and closest node to random point so cinfigured are marked with a red dot, as A and are joined with dotted line.
This point A is the new added node to the tree and the also edge representing the trajectory between tree node and point A is added.

* ![primitive2](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/primitive_2.JPG)

 * For the third module, a RRT with these primitive motion contrainst were implemented which can be found [here](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/RRT_NonHolonomic_R2S1.m),which looks like:
 
 ![Combined RRT](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/0.jpg)
 
* **After this bias along the staright path from the initial to the final confiuration was added as:**
*
```
    random_number = rand;
    bias = 0.1; %Probability by which goal node will be sampled
    if random_number >= (1 - bias) && random_number < 1
        x_rand = x_goal;
        y_rand = y_goal;
        theta_rand = theta_goal;
    else
        x_rand = round((x_max - x_min)*rand);
        y_rand = round((y_max - y_min)*rand);
        theta_rand = (2*pi - 0.0001)*rand;
    end
```
 
 * third module with the incorporation on the bias, looks like:
 ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/result_with_bias.JPG)
 
 * Just for Visualization purpose I have also drawn the orientation of the car at a particular point. Here, the point ends where the black line ends and starts with a red colored line vector representing the orientation of the car. 
 ```
 ed = [cos(theta_new) -sin(theta_new); sin(theta_new) cos(theta_goal)] * [10*1.5; 0];
 plot([x_new, x_new+ed(1)], [y_new, y_new+ed(2)], 'r-', 'Linewidth', 2);
 plot(x_new+ed(1), y_new+ed(2), 'ko', 'MarkerSize',5, 'MarkerFaceColor','k');
 ```
 * This can be visualized as:
 ![](https://github.com/architkhullar/RobotMotionPlanning_TermProject/blob/master/Images/result_with_orientation.JPG)
 
**Running time of the algorithm n^2**
 
### Incomplete things:
* **Though my random samples are only generating at the white spaces the collison detection is not working and, i.e. it is still making matks through the configuration space. I was trying rectify this problem by creating array of obstacles map and then while finding the nearest node to the randomly generated point, it check if they are intersecting. If they, intersect it moves to the next nearest and so on**  
  
### Scope of Improvement:
* I should have stuck to my own Strength, instead of getting into a unexplored territory of MATLAB, Should have stuck to C++
* I didn't take into account the collision detecting algorithms while starting it and hence it complicated the code later.
* while starting, for a very long time I tried to do it with MATLAB's inbuilt library called Automated System Driving Toolbox, which had a lot of constraints pre defined [link](https://www.mathworks.com/products/automated-driving/features.html)

### Limitations of the project:
* No collision detection
* No bias for the random generation of points **(This is also fixed)** and the theta or the angle of the orientation and hence very slow
* can work more to optimizethe orientation (can add a bias for the orientations as well)
* Optimized step size
* Biases are always tricky and need a lot of experimentation, which I wasn't left with much.
	
