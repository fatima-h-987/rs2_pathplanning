#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>
#include <vector>
#include <map>
#include <fstream>


//ROS data types
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/GetPlan.h>


//We include header of another class we are developing
#include "laserprocessing.h"


class Sample
{
public:
  /// @brief Constructor of the Sample class.
  ///
  /// Sets the default values of variables such as the robot position, the goals, the running_ boolean, etc.
  /// Requires the NodeHandle input to communicate with ROS.
  Sample(ros::NodeHandle nh);

  /// @brief Destructor of the Sample class.
  ///
  /// Deletes the object pointers for laserprocessing and imageprocessing classes.
  ~Sample();

  // CHANGE THIS WHEN NEEDED
  static const int NumOfDest = 40; // including origin

  std::vector<geometry_msgs::Point> planBetweenTwoGoals(geometry_msgs::Point st, geometry_msgs::Point en, int n);
  
  /// @brief seperate thread.
  void generateRandomGoals();
  
  /// @brief seperate thread.
  void generateRandomGoal();
  
  /// The MAIN PROCESSING THREAD that will run continously and utilise the data.
  /// When data needs to be combined then running a thread seperate to callback will guarantee data is processed.
  /// The data is then used to publish input to move the TurtleBot, causing new data to be generated on its updated position and perspective.
  bool isGoalValid(double x, double y);


  void publishPath(std::vector<geometry_msgs::Point> vec_of_simplified_waypts);

  void orderGoalsUsingTSP();

  void findMinRoute(std::vector<std::vector<double> > tsp);

  void writePointsToCSV(std::vector<geometry_msgs::PoseStamped> poses, const std::string& filename);

  /// The MAIN PROCESSING THREAD that will run continously and utilise the data.
  /// When data needs to be combined then running a thread seperate to callback will guarantee data is processed.
  /// The data is then used to publish input to move the TurtleBot, causing new data to be generated on its updated position and perspective.
  void seperateThread();
  
  double DistanceToGoal(double goal_x, double goal_y, geometry_msgs::Pose robot);

  double totalDistanceFromPlan(geometry_msgs::Point g1, geometry_msgs::Point g2);

  double DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2);

  void publishMarkers(geometry_msgs::Point point, double time, bool waypt);

  visualization_msgs::Marker createMarker(double x, double y, double time, bool waypt);

  double fabs(double x); // delete

  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
private:
  //! Node handle for communication
  ros::NodeHandle nh_;
  //! Goal publisher 
  ros::Publisher goal_pub_;
  //! Goals publisher
  ros::Publisher goals_pub_;
  //! Where Path NEEDED is published
  ros::Publisher custompath_pub_;
  // random goals publisher
  ros::Publisher goal_marked_pub_;
  // visulisation marker 
  ros::Publisher vis_pub_;


  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub_scan;
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub_odom;
  //! Map subscribe
  ros::Subscriber map_sub_;
  //! Make Plan
  ros::ServiceClient make_plan_;
  
  // stores bit map 
  nav_msgs::OccupancyGrid current_map;
  // vector with minimum paht weights
  std::vector<int> saved_perm_;


  //! Pointer to Laser Object
  LaserProcessing* laserProcessingPtr_;
  // laser data
  sensor_msgs::LaserScan laserData_;
  //! Mutex to lock laserData_
  std::mutex laserDataMtx_;
  //! Stores the position and orientation of the robot
  geometry_msgs::Pose robotPose_;
  //! Mutex to lock robotPose_
  std::mutex robotPoseMtx_;

    double threshold_distance_;
    int map_width_;
    int map_height_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    std::vector<int8_t> map_data_;
    double world_x;
    double world_y;
    std::vector<geometry_msgs::PoseStamped> unordered_goals_;
    std::vector<geometry_msgs::Point> ordered_goals_;
    geometry_msgs::PoseStamped currentgoal_;
    unsigned int markerCounter_;
    int waypointssize_;
    int NumOfDestinations_; // do not need to use
    bool canReachGoal_;
  //! Stores a goal for the robot to move towards
  geometry_msgs::Point goal_;
  //! Stores a series of goals in the order they occur
  std::vector<geometry_msgs::Point> goals_;

};

#endif // SAMPLE_H