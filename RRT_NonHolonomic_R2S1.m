clc; clear all; close all;
%% Setting up the environment

N = 5000; %maximum number of iterations
step_size = 10;
distance_threshold = 40; %maximum distance between random node and the goal state
orientation_threshold = 10*pi/180;

% Set vehicle constants
dt = 0.1; % Step by 0.1 seconds
L = 7.5; % Car length
max_steering_angle = 0.7; %radians
max_linear_vel = 40;

x_init = 100; y_init = 100; theta_init = 0;
x_goal = 500; y_goal = 400; theta_goal = 90*pi/180;

% Enter the map here
rectangle('Position', [40 0 10 80], 'FaceColor', 'black');
rectangle('Position', [90 60 10 100], 'FaceColor', 'black');
rectangle('Position', [90 160 70 10], 'FaceColor', 'black');
rectangle('Position', [150 60 10 100], 'FaceColor', 'black');
rectangle('Position', [120 0 10 40], 'FaceColor', 'black');

% ## MAP 2 ##
rectangle('Position', [0 40 120 10], 'FaceColor', 'black');
rectangle('Position', [80 90 180 10], 'FaceColor', 'black');
rectangle('Position', [0 140 120 10], 'FaceColor', 'black');
mapMatrix(1:120, 40:50) = 1;
mapMatrix(80:200, 90:100) = 1;
mapMatrix(1:120, 140:150) = 1;

mapMatrix(40:50, 1:80) = 1;
mapMatrix(90:100, 60:160) = 1;
mapMatrix(90:160, 160:170) = 1;
mapMatrix(150:160, 60:160) = 1;
mapMatrix(120:130, 1:40) = 1;

%grid size
x_min = 0; y_min = 0;
x_max = 600; y_max = 600;


%drawing the initial and the goal position
figure(1); axis([x_min x_max y_min y_max]); hold on; grid on;
plot(x_init, y_init, 'ko', 'MarkerSize',5, 'MarkerFaceColor','k');
plot(x_goal, y_goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');

%line for initial orientation at intitial position
ed = [cos(theta_init) -sin(theta_init); sin(theta_init) cos(theta_init)] * [20*1.5; 0];
plot([x_init, x_init+ed(1)], [y_init, y_init+ed(2)], 'y-', 'Linewidth', 3);
plot(x_init+ed(1), y_init+ed(2), 'ko', 'MarkerSize',5, 'MarkerFaceColor','b');

%line for final orientation at final position
ed = [cos(theta_goal) -sin(theta_goal); sin(theta_goal) cos(theta_goal)] * [20*1.5; 0];
plot([x_goal, x_goal+ed(1)], [y_goal, y_goal+ed(2)], 'y-', 'Linewidth', 3);
plot(x_goal+ed(1), y_goal+ed(2), 'ko', 'MarkerSize',5, 'MarkerFaceColor','b');

%Drawing threshold region around the goal position
viscircles([x_goal, y_goal],distance_threshold,'LineStyle','--');

%% The data structure used to build RRT is a nested structure

%initializing tree
rrt_tree.node(1).x = x_init;
rrt_tree.node(1).y = y_init;
rrt_tree.node(1).theta = theta_init;
rrt_tree.node(1).x_parent = x_init;
rrt_tree.node(1).y_parent = y_init;
rrt_tree.node(1).theta_parent = theta_init;

%velocity at the parent node
rrt_tree.node(1).parent_velocity = 0;

%steering angle at the parent node
rrt_tree.node(1).parent_steering_angle = 0; 
rrt_tree.node(1).distance = 0;
rrt_tree.node(1).index = 1; 
rrt_tree.node(1).parent_index = 0;

%% Building Nonholonomic RRT

%initializing x,y-coordinate and orientation theta of new node that will be
%added in tree R2 * S1
x_new = 0; y_new = 0; theta_new = 0;

iter = 2;
while iter <= N
    %% Generating uniformly distributed random configurations
    
    %Random configuration from a uniform distribution
    x_rand = round((x_max - x_min)*rand);
    y_rand = round((y_max - y_min)*rand);
    theta_rand = (2*pi - 0.0001)*rand;
    
    
    %% calculating distance between generated random node and other nodes in the tree
    for i = 1:length(rrt_tree.node)
        %Taking euclidean distance metric
        distance(i) = sqrt( (x_rand - rrt_tree.node(i).x)^2 + (y_rand - rrt_tree.node(i).y)^2 +  ((180/pi)^2)*min( [ (theta_rand - rrt_tree.node(i).theta)^2, (theta_rand - rrt_tree.node(i).theta - pi)^2, (theta_rand - rrt_tree.node(i).theta + pi)^2 ] )   );
    end
    [distance_value, parent_index] = min(distance);
    
    %Node in that is closes to the random configuration
    %This is the parent node for the node that will be added to the tree
    x_near = rrt_tree.node(parent_index).x;
    y_near = rrt_tree.node(parent_index).y;
    theta_near = rrt_tree.node(parent_index).theta;
    
    %% Computing motion primitives from the parent node and storing the end points
    % of the trajectories as new possible nodes in the new_node_array
    
    path = [x_near, y_near, theta_near]; %parent node in tree from where we calculate the paths
    steering_angle = -max_steering_angle;
    linear_vel = -max_linear_vel;
    new_node_array_iter = 1; %iterator to keep track of the number of element being added
    distance_new_node = 0; %distance of end point of the motion primitive from random node
    while linear_vel <= max_linear_vel
        while steering_angle <= max_steering_angle
            for i = 2:step_size
                path(i,1) = path(i-1,1) + linear_vel*cos(path(i-1,3))*dt;
                path(i,2) = path(i-1,2) + linear_vel*sin(path(i-1,3))*dt;
                path(i,3) = path(i-1,3) + (linear_vel/L)*tan(steering_angle)*dt;
            end
            distance_new_node = sqrt( (x_rand - path(i,1))^2 + (y_rand - path(i,2))^2 + ((180/pi)^2)*min( [ (theta_rand - path(i,3))^2, (theta_rand - path(i,3) - pi)^2, (theta_rand - path(i,3) + pi)^2 ] )   );
            new_node_array(new_node_array_iter,:) = [distance_new_node, linear_vel, steering_angle, path(i,1), path(i,2), path(i,3)];
            new_node_array_iter = new_node_array_iter + 1;

            steering_angle = steering_angle + 0.05;
        end
        if (linear_vel + 5) > -30 && (linear_vel + 5) < 30
            linear_vel = 30;
        else
            linear_vel = linear_vel + 5;
        end
        steering_angle = -max_steering_angle; %Resetting steering angle for the next linear velocity
    end
    
    %% Computing which node from the new possible nodes is closest to the
    % random node
    [distance_new_node_value, new_node_array_index] = min(new_node_array(:,1));
    %new node parameters
    x_new = new_node_array(new_node_array_index,4);
    y_new = new_node_array(new_node_array_index,5);
    theta_new = new_node_array(new_node_array_index,6);
    
    %Making sure that value of theta lies between 0 and 2*pi. 
    if theta_new < 0
        theta_new = 2*pi - abs(theta_new);
    end
    if theta_new > 2*pi
        while theta_new > 2*pi
            theta_new = theta_new - 2*pi;
        end
    end
    
    parent_vel_new = new_node_array(new_node_array_index,2); %Velocity needed at parent node to reach the new node
    parent_steering_angle_new = new_node_array(new_node_array_index,3); %Steering angle needed at the parent node to reach new node
    %This is because while calculating the paths above we calculate the
    %paths from the parent node. Thus velocity and steering angle are the
    %values at parent node.
    
    %% Checking if the new node lies within the limits of the grid
    
    new_node_validity = 0; %zero if node lies within grid and 1 if it doesn't
    new_x_node_validity = 0;
    new_y_node_valifity = 0;
    if x_new <= x_min || x_new >= x_max
        new_node_validity = 1;
    elseif y_new <= y_min || y_new >= y_max
        new_node_validity = 1;
    end
    
    %% Adding the new node the tree and drawing the path between parent node and new node
    
    if new_node_validity == 0
        rrt_tree.node(iter).x = x_new;
        rrt_tree.node(iter).y = y_new;
        rrt_tree.node(iter).theta = theta_new;
        rrt_tree.node(iter).x_parent = x_near;
        rrt_tree.node(iter).y_parent = y_near;
        rrt_tree.node(iter).theta_parent = theta_near;
        rrt_tree.node(iter).parent_velocity = parent_vel_new; 
        rrt_tree.node(iter).parent_steering_angle = parent_steering_angle_new; 
        rrt_tree.node(iter).distance = distance_new_node_value;
        rrt_tree.node(iter).index = iter; 
        rrt_tree.node(iter).parent_index = parent_index;
        
        %Drawing the path between the new node and the parent node
        path = [x_near, y_near, theta_near];
        for i = 2:step_size
            path(i,1) = path(i-1,1) + parent_vel_new*cos(path(i-1,3))*dt;
            path(i,2) = path(i-1,2) + parent_vel_new*sin(path(i-1,3))*dt;
            path(i,3) = path(i-1,3) + (parent_vel_new/L)*tan(parent_steering_angle_new)*dt;
        end
        plot(path(i,1), path(i,2), 'ko', 'MarkerSize',2, 'MarkerFaceColor','b');
        for j = 2:size(path,1)
            line([path(j,1), path(j-1,1)], [path(j,2), path(j-1,2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
        end
        
      
        %Checking if goal is within threshold limits from the new state
        distance_to_goal = sqrt( (x_new - x_goal)^2 + (y_new - y_goal)^2); 
        if distance_to_goal <= distance_threshold && ( abs(theta_new - theta_goal)<=orientation_threshold || abs(theta_new - theta_goal - pi) <=orientation_threshold )
            disp('Final Configuration Near the Goal Configuration is:')
            x_coordinate = x_new
            y_coordinate = y_new
            orientation_in_degrees = theta_new*180/pi
            break;
        end
        
        %iterator for the main while loop counting the number of iterations
        iter = iter + 1;
    else
        continue
    end
end

%% plotting the path from goal to the start by backtracing
index = length(rrt_tree.node);
parent_index = rrt_tree.node(index).parent_index;

if sqrt((rrt_tree.node(index).x - x_goal)^2 + (rrt_tree.node(index).y - y_goal)^2) <= distance_threshold
    while parent_index ~= 0
        %while backtracking the path we draw path from parent node to the
        %child node
        path = [rrt_tree.node(index).x_parent, rrt_tree.node(index).y_parent, rrt_tree.node(index).theta_parent ];
        parent_vel_new = rrt_tree.node(index).parent_velocity;
        parent_steering_angle_new = rrt_tree.node(index).parent_steering_angle;
        for i = 2:step_size
            path(i,1) = path(i-1,1) + parent_vel_new*cos(path(i-1,3))*dt;
            path(i,2) = path(i-1,2) + parent_vel_new*sin(path(i-1,3))*dt;
            path(i,3) = path(i-1,3) + (parent_vel_new/L)*tan(parent_steering_angle_new)*dt;
        end
        for j = 2:size(path,1)
            line([path(j,1), path(j-1,1)], [path(j,2), path(j-1,2)], 'Color', 'r', 'LineWidth', 4);
            drawnow
            hold on
        end
        index = parent_index;
        parent_index = rrt_tree.node(index).parent_index;
    end
end
%% Reading the elapsed time from stopwatch started by tic
toc 