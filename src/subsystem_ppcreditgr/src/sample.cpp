#include "sample.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <random>
#include <bits/stdc++.h>


//Default constructor of the sample class
Sample::Sample(ros::NodeHandle nh) :
    //Setting the default value for some variables
    nh_(nh), laserProcessingPtr_(nullptr), world_x(0.0), world_y(0.0), markerCounter_(0), 
    waypointssize_(0), canReachGoal_(true)
{
    //Subscribing to the laser sensor
    sub_scan = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    //Subscribing to odometry of the robot
    sub_odom = nh_.subscribe("/odom", 100, &Sample::odomCallback,this);
    map_sub_ = nh_.subscribe("/map", 100, &Sample::mapCallback, this);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    custompath_pub_ = nh_.advertise<geometry_msgs::Point>("/thepath", 10);
    goal_marked_pub_ = nh_.advertise<geometry_msgs::Point>("/therandomgoals", 10);
    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",3,false);
    //goals_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/randomgoals", 5);

    make_plan_ = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    //NumOfDest = 5; // so 4 random generated goals
  }

// We delete anything that needs removing here specifically
Sample::~Sample(){
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}

//A callback for the laser scanner
void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(laserDataMtx_); // Locks the data for the laserData to be saved
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
}

//A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    std::unique_lock<std::mutex> lck(robotPoseMtx_); // Locks the data for the robotPose to be saved
    robotPose_ = pose; // We copy the pose here
}

//A callback for map
void Sample::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
         // Parse the map data
        map_width_ = msg->info.width;
        //std::cout << "Map Width: " << map_width_ << std::endl;
        map_height_ = msg->info.height;
        //std::cout << "Map Height: " << map_height_ << std::endl;
        map_resolution_ = msg->info.resolution;
        //std::cout << "Map Resolution: " << map_resolution_ << std::endl;
        map_origin_x_ = msg->info.origin.position.x;
        //std::cout << "Map Origin X: " << map_origin_x_ << std::endl;
        map_origin_y_ = msg->info.origin.position.y;
        //std::cout << "Map Origin Y: " << map_origin_y_ << std::endl;
        map_data_ = msg->data;
        //std::cout << "Map data provided." << std::endl;
        // Generate a random goal point
}

void Sample::generateRandomGoals()
{
    unordered_goals_.clear(); // does not have initial pose - to fix
    ordered_goals_.clear(); // does have initial pose
    saved_perm_.clear();

    geometry_msgs::PoseStamped initial;
    initial.pose.position.x = 0; // Set the goal position x-coordinate
    initial.pose.position.y = 0; // Set the goal position y-coordinate
    initial.pose.orientation.w = 1.0;
    initial.header.stamp = ros::Time::now();
    initial.header.frame_id = "map";
    // Push goal message into vector
    //ordered_goals_.push_back(initial.pose.position);
    unordered_goals_.push_back(initial); // this will be fed into algorithm
    std::cout << "Initial pose has been pushed into vector." << std::endl;
    goal_marked_pub_.publish(initial.pose.position);
    
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    std::vector<geometry_msgs::Point> waypts_simplified; // the vector created by plan between two Goals
    
    // UNCOMMENT WHEN DONE
    for (int i=0; i<NumOfDest-1; i++)
    {
        // push a random goal into the unordered_goals_ vector
        generateRandomGoal(); 
    }
    std::cout << NumOfDest << " goals generated!!" << std::endl;

    // COMMENT CSV REFERENCE HERE
    //writePointsToCSV(unordered_goals_, "/home/ubuntu/fatimahaq/rs2_individual_test/src/subsystem_ppcreditgr/src/points.csv");
    orderGoalsUsingTSP();

    // Make plans between each goal (now ordered)
    for(int g = 0; g < ordered_goals_.size()-1; ++g)
    {
        start = ordered_goals_[g];
        end = ordered_goals_[g+1];
        
        // for error checking
        std::cout << "Waypoints plotted from Goal " << g <<  " {" << start.x << " , " << start.y << "} to"
        << " Goal " << g+1 << " {" << end.x << " , " << end.y << "}." << std::endl; 
        
        // find simplified path
        waypts_simplified = planBetweenTwoGoals(start, end, 5);
        // publish to rostopic 'thepath'
        publishPath(waypts_simplified);
    }
}

void Sample::writePointsToCSV(std::vector<geometry_msgs::PoseStamped> poses, const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (const auto& pose : poses) {
        // Write x and y values in separate columns
        outfile << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
    }
    outfile.close();
}


void Sample::orderGoalsUsingTSP()
{
    const int V = NumOfDest; // Number of vertices
    //std::vector<std::vector<int>> tsp;
    std::vector<std::vector<double>> tsp(V, std::vector<double>(V, -1));

    // Calculate distances between each pair of goals
    int i;
    int j;
    for (i = 0; i < V; i++)
    {
        for (j = 0; j < V; j++)
        {
            if (i == j) {
                tsp[i][j] = -1; // Distance to itself is 0
            } 
            else
            {
                // Calculate straight-line distance between goals i and j
                tsp[i][j] = totalDistanceFromPlan(unordered_goals_[i].pose.position, unordered_goals_[j].pose.position);

            }
        }
    }
    std::cout << "TSP graph created." << std::endl;

    // PRINT Matrix
    // std::cout << "Printing vector of..." << std::endl;
    // for (int a = 0; a < NumOfDest; ++a) {
    //     for (int b = 0; b < NumOfDest; ++b) {
    //         std::cout << tsp[a][b] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // PRINT VECTOR
    std::cout << "Printing unordered goals vector:" << std::endl;
    for (const auto& pose : unordered_goals_) {
        std::cout << "(" << pose.pose.position.x << ", " << pose.pose.position.y << ")" << std::endl;
    }

    geometry_msgs::Point origin;
    origin.x = 0; 
    origin.y = 0;
    ordered_goals_.push_back(origin);


    findMinRoute(tsp); // CHANGE THIS 

    for(int y = 0; y < saved_perm_.size(); y++) // used to have +1 but removed to avoid segmt. fault
    {
        int order = saved_perm_[y]; // the order of yth goal in unordered vector
        geometry_msgs::Point pt = unordered_goals_[order].pose.position; // the point on yth
        ordered_goals_.push_back(pt);
    }

    geometry_msgs::Point backtoorigin;
    backtoorigin.x = 0; 
    backtoorigin.y = 0;
    ordered_goals_.push_back(backtoorigin);

    // PRINT VECTOR
    std::cout << "Printing ordered goals vector:" << std::endl;
    for (const auto& point : ordered_goals_) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }

}



void Sample::findMinRoute(std::vector<std::vector<double> > tsp)
{
    double sum = 0;
    int counter = 0;
    int j = 0, i = 0;
    double min = INT_MAX;
    std::map<int, int> visitedRouteList;
 
    // Starting from the 0th indexed
    // city i.e., the first city
    visitedRouteList[0] = 1;
    int route[tsp.size()];
 
    // Traverse the adjacency
    // matrix tsp[][]
    while (i < tsp.size() && j < tsp[i].size())
    {
 
        // Corner of the Matrix
        if (counter >= tsp[i].size() - 1)
        {
            break;
        }
 
        // If this path is unvisited then
        // and if the cost is less then
        // update the cost
        if (j != i && (visitedRouteList[j] == 0))
        {
            if (tsp[i][j] < min)
            {
                min = tsp[i][j];
                route[counter] = j + 1;
            }
        }
        j++;
 
        // Check all paths from the
        // ith indexed city
        if (j == tsp[i].size())
        {
            sum += min;
            min = INT_MAX;
            visitedRouteList[route[counter] - 1] = 1;
            j = 0;
            i = route[counter] - 1;
            counter++;
        }
    }
 
    // Update the ending city in array
    // from city which was last visited
    i = route[counter - 1] - 1;
 
        for (j = 0; j < tsp.size(); j++)
        {
    
            if ((i != j) && tsp[i][j] < min) 
            {
                min = tsp[i][j];
                route[counter] = j + 1;
            }
        }
        sum += min;

    
    
    // Started from the node where
    // we finished as well.
    geometry_msgs::Point pt;
    std::cout << "Route (excluding origin): ";
    for (int k = 0; k < counter; k++) // used ot be <=
    {
        std::cout << route[k] - 1 << " ";
        int order = route[k] - 1; // the order of yth goal in unordered vector
        pt = unordered_goals_[order].pose.position; // the point on yth
        ordered_goals_.push_back(pt);
    }
    std::cout << std::endl;
    geometry_msgs::Point origin;
    origin.x = 0; origin.y = 0;
    double last = totalDistanceFromPlan(pt, origin);
    sum = sum + last;

    std::cout << ("PATHWEIGHT CALCULATED: ") << sum << std::endl;


}



void Sample::publishPath(std::vector<geometry_msgs::Point> vec_of_simplified_waypts)
{
   for (const auto &point : vec_of_simplified_waypts)
    {
        custompath_pub_.publish(point);
        publishMarkers(point, 1000, 1); // 1 means it is a waypoint
        std::this_thread::sleep_for(std::chrono::milliseconds(7)); //10

    }
    // wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); //50
}



std::vector<geometry_msgs::Point> Sample::planBetweenTwoGoals(geometry_msgs::Point st, geometry_msgs::Point en, int n)
{
    std::vector<geometry_msgs::Point> points;
  // Create a request message for the service
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = st.x;
    srv.request.start.pose.position.y = st.y;
    srv.request.start.pose.orientation.w = 1.0;

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = en.x;
    srv.request.goal.pose.position.y = en.y;
    srv.request.goal.pose.orientation.w = 1.0;

    // Call the service - the statement within the if() calls the service with srv
    if (make_plan_.call(srv)) {
        if (!srv.response.plan.poses.empty())
        {
            //ROS_INFO("Plan received with %ld poses", srv.response.plan.poses.size());
            for (size_t i = 0; i < srv.response.plan.poses.size(); i += n)
            {   
                points.push_back(srv.response.plan.poses[i].pose.position);
                double x = srv.response.plan.poses[i].pose.position.x;
                double y = srv.response.plan.poses[i].pose.position.y;
                //std::cout << "Position: (" << x << " , " << y << ")." << std::endl; 
            }
            // if last point is not end point / goal
            if(!(points.back()==en))
            {
                //std::cout << "Since point vector doesn't have end goal as last point, pushing it back." << std::endl; 
                points.push_back(en);
                //std::cout << "Position: (" << en.x << " , " << en.y << ")." << std::endl; 
            }
            canReachGoal_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //25

        } 
        else 
        {
            //ROS_WARN("Received an empty plan so removing last random goal. Generating new goal...");
            canReachGoal_ = false;
        }
    } 
    else
    {
        ROS_ERROR("Failed to call service make_plan");
    }

    waypointssize_ = points.size();
    //std::cout << "Number of elements in the simplified points vector is: " << waypointssize_ << std::endl;
    return points;
}


void Sample::generateRandomGoal()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); //500
    uint32_t idx;

        while (true) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> grid_x(10, map_width_-10);
            std::uniform_real_distribution<> grid_y(10, map_height_-10);
            double map_x = grid_x(gen);
            double map_y = grid_y(gen);

            if (Sample::isGoalValid(map_x, map_y))
            {   
                //std::cout << "map_x: " << map_x << ", map_y: " << map_y << "} is a valid goal." << std::endl;
                //std::cout << " Random Goal { " << world_x << " , " << world_y << " } is valid." << std::endl;
                break;
            }
        }

        // Put the goal point in PoseStamped format
        currentgoal_.pose.position.x = world_x; // Set the goal position x-coordinate
        currentgoal_.pose.position.y = world_y; // Set the goal position y-coordinate
        currentgoal_.pose.orientation.w = 1.0;
        currentgoal_.header.stamp = ros::Time::now();
        currentgoal_.header.frame_id = "map";

                // Push goal message into vector
        unordered_goals_.push_back(currentgoal_);
        goal_marked_pub_.publish(currentgoal_.pose.position);
        publishMarkers(currentgoal_.pose.position, 1000, 0);
        std::cout << "Goal {" << world_x << " , " << world_y << " } has been pushed into vector." << std::endl;


}

bool Sample::isGoalValid(double x, double y)
{
    world_x = map_origin_x_ + (x) * map_resolution_; // (x+ 0.5)
    world_y = map_origin_y_ + (y) * map_resolution_;
    // Check if the goal point is a threshold distance away from obstacles
        //int map_x = (x - map_origin_x_) / map_resolution_;
        //int map_y = (y - map_origin_y_) / map_resolution_;
        uint32_t idx = x + y * map_width_;
        // std::cout << "Index is: " << idx << std::endl;
        //int index = map_y * map_width_ + map_x;
        if (idx >= 0 && idx < map_data_.size()) 
        {
            bool withinBounds = false;
            bool freeSpace = false;
            bool neighboursUnoccupied = true; // assume unoccupied 
            bool withinThreshold = false;
            if(world_x >= 0 && world_x <= 10 && world_y >= -9 && world_y <= 0)
            {
                withinBounds = true;
            }
            if (map_data_[idx] == 0)
            {
                freeSpace = true;
            }
            for (int i = idx-100; i <= idx+100; ++i) // for data 100 to right and 100 to left
            {
                if(!(map_data_[idx] == 0))
                {
                    neighboursUnoccupied = false;
                    break;
                }
            }
            if(DistanceToGoal(world_x, world_y, robotPose_) > 1)
            {
                withinThreshold = true;
            }

        // now check if a path to the goal generated can be mapped
        // bool canReachGoal_ used!
        geometry_msgs::Point origin;
        origin.x = 0.0; // Set the goal position x-coordinate
        origin.y = 0.0; // Set the goal position y-coordinate

        geometry_msgs::Point checkpoint;
        checkpoint.x = world_x; // Set the goal position x-coordinate
        checkpoint.y = world_y; // Set the goal position y-coordinate

        std::vector<geometry_msgs::Point> checkpath; 
        checkpath = planBetweenTwoGoals(origin, checkpoint, 2);

            if(canReachGoal_)
            {
                if(withinBounds && freeSpace && neighboursUnoccupied && withinThreshold)
                {
                    //std::cout <<  "If 0, free space: " << map_data_[idx] << std::endl; 
                    //std::cout << "Goal VALID as free space, within bounds and distance threshold and neighbours cells are unoccupied." << std::endl;
                    return true;
                }
                else
                {
                    //std::cout << "Goal inside obstacle, out of bounds or distance threshold, unknown, or neigbouring cells occupied." << std::endl;
                    return false; // Goal point is inside an obstacle
                    generateRandomGoal(); // generate another random goal
                }
            }
            else
            {
                return false;
                generateRandomGoal();
            }
    }
}

//Gets the distance from the goal to the robot
double Sample::DistanceToGoal(double goal_x, double goal_y, geometry_msgs::Pose robot)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal_x-robot.position.x,2)+pow(goal_y-robot.position.y,2));
    return dist;
}

double Sample::totalDistanceFromPlan(geometry_msgs::Point g1, geometry_msgs::Point g2)
{
    double totaldist = 0;
    std::vector<geometry_msgs::Point> path; 
    path = planBetweenTwoGoals(g1, g2, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    if(waypointssize_<2) // to ensure a plan is made
    {
        path = planBetweenTwoGoals(g1, g2, 10);
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if(waypointssize_<2) // to ensure a plan is made
    {
        path = planBetweenTwoGoals(g1, g2, 2);
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        double distance = DistanceBetweenGoals(path[i], path[i + 1]);
        totaldist = totaldist + distance;     
        //std::cout << "total dist is: " << totaldist << std::endl;
    } 
    return totaldist;
}


//Gets the distance from two goals
double Sample::DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    //double dist_int;
    double dist = sqrt(pow(goal1.x-goal2.x,2)+pow(goal1.y-goal2.y,2));
    return dist;
}

void Sample::seperateThread() {
    //Waits for the data to be populated from ROS
    while(laserData_.range_min+laserData_.range_max == 0.0);//||
        //   robotPose_.orientation.w+robotPose_.orientation.x+
        //   robotPose_.orientation.y+robotPose_.orientation.z == 0.0);

    //Limits the execution of this code to 5Hz
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {
        //Locks all of the data with mutexes
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (robotPoseMtx_);
        
        //Creates the class object and gives the data from the sensors
        LaserProcessing laserProcessing(laserData_);

        //Unlocks all mutexes
        lck2.unlock();
        lck1.unlock();

        // can put code here 

        //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
        //it will do it for the exact amount of time needed to run at 5Hz
        rate_limiter.sleep();
    }
}

double Sample::fabs(double x)
{
    if(x < 0) return -x;
    else return x;
}


void Sample::publishMarkers(geometry_msgs::Point point, double time, bool waypt)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); //100
    // send to createMaker() function to convert to marker type and set duration 
    visualization_msgs::Marker marker = createMarker(point.x, point.y, time, waypt);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    // publish the marker array
    vis_pub_.publish(marker_array);
}


visualization_msgs::Marker Sample::createMarker(double x, double y, double time, bool waypt)
{

    visualization_msgs::Marker marker; // creating a marker 
    marker.header.frame_id = "map"; // set the frame ID
    marker.header.stamp = ros::Time::now();

    // setting name, unique ID, lifetime and type
    marker.lifetime = ros::Duration(time); 
    marker.ns = "point"; 
    marker.id = markerCounter_++; 

    marker.scale.z = 0.0;
    std_msgs::ColorRGBA color;
    // alpha is stransparency (50% transparent)
    color.a = 1.0;
    if(waypt==0) // then goal
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.22;
        marker.scale.y = 0.22;     // colour is yellow
        color.r = 1.0;
        color.g = static_cast<float>(255.0/255.0);
        color.b = 0.0;
    }
    else // waypoint 
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        color.r = 1.0;
        color.g = 0;
        color.b = 0; // colour is purple
    }
    marker.color = color;


    marker.action = visualization_msgs::Marker::ADD; // ADD marker to screen
    // setting position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    // setting orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side

    return marker;
}