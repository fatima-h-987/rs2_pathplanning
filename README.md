# rs2_pathplanning
Art Gallery TurtleBot3 Path Planning by Fatima Haq (SID:13891724). All code is in C++. 

## Pre-requisites 
Step 1: Download all TurtleBot3 packages and dependencies. 

Step 2: Download the Simple Fast Multimedia Library (SFML) software using following commands:
    
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install libsfml-dev 
    sudo apt-get update

When logging in or saving edits in respective folders, follow below commands:
    
    cd catkin_ws
    catkin_make

When logging in or saving edits in TSP folders/files, follow below commands:
    
    cd src/TSP
    ./build.sh

## Brute Force
The 'subsystem_ppcredit' package employs the brute force to solve the travelling salesman problem. 

By default, the number of destinations (including origin) will be 9, meaning 8 random goals will be generated. Line 44 of sample.h can be changed to increase destinations. Please note that more than 10 destinations will take a long time to path plan around due to the nature of the brute force.

After running catkin_make, follow the below commands. In a new terminal, run:
    
    roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch 
In another terminal, run:
    
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml 
In the navigation.rviz, click the 'Add' button on the bottom left. Add 'MarkerArray'. After adding this, it is desirable to increase the queue size to 5000. In a third terminal, run:
    
    rosrun subsystem_ppcredit subsystem_ppcredit_test
Ctrl+C once finished. 

## Greedy Approach
The 'subsystem_ppcreditgr' package employs the greedy approach to solve the travelling salesman problem. 

By default, the number of destinations (including origin) will be 40. Line 47 of sample.h can be changed to increase destinations. Please note that more than 50 destinations will take a long time to path plan around.

After running catkin_make, follow the below commands. In a new terminal, run:
    
    roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch 
In another terminal, run:
    
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml 
In the navigation.rviz, click the 'Add' button on the bottom left. Add 'MarkerArray'. After adding this, it is desirable to increase the queue size to 5000. In a third terminal, run:
    
    rosrun subsystem_ppcreditgr subsystem_ppcreditgr_test  
Ctrl+C once finished. 

## Self-Organising Map Approach
The TSP folder employs the brute force to solve the travelling salesman problem. 

By default, the number of destinations (including origin) will be 200. The 'run.sh' executable file under the TSP folder refers to the 200points csv file under the Data folder. Goals on the map have already been generated randomly to avoid excessive time.  200 can be change to 500 if desired.


After saving, remember to run ./build.sh:
    
    cd src/TSP
    ./build.sh
Execute the following command in the same terminal.
    
    ./run.sh  
Ctrl+C once finished. 

## Repositories / References Used

https://wiki.ros.org/move_base

https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/

https://www.geeksforgeeks.org/travelling-salesman-problem-greedy-approach/

https://github.com/martisalcedo7/TSP/tree/main

https://www.sfml-dev.org/tutorials/2.5/start-linux.php

