#include <iostream>
#include <math.h>
#include <vector>
#include <cassert>
#include "cities.hpp"
#include "utils.hpp"

Cities::Cities(const std::vector<Point> &cities_vector){

    cities = cities_vector;
    number_of_cities = cities.size();

    min_coordinates.x = cities.at(0).x;
    max_coordinates.x = cities.at(0).x;
    min_coordinates.y = cities.at(0).y;
    max_coordinates.y = cities.at(0).y;

    for(size_t i=1; i<number_of_cities; i++){
        if(cities.at(i).x < min_coordinates.x){min_coordinates.x = cities.at(i).x;};
        if(cities.at(i).x > max_coordinates.x){max_coordinates.x = cities.at(i).x;};
        if(cities.at(i).y < min_coordinates.y){min_coordinates.y = cities.at(i).y;};
        if(cities.at(i).y > max_coordinates.y){max_coordinates.y = cities.at(i).y;};
    }

    calculate_distance_matrix();
}

void Cities::calculate_distance_matrix(void){
    std::vector<std::vector<float>> matrix(number_of_cities, std::vector<float>(number_of_cities, 0));
    for(size_t c_1 = 0; c_1 < number_of_cities; c_1++) {
        for(size_t c_2 = c_1; c_2 < number_of_cities; c_2++) {
            if(c_1 == c_2){
                continue;
            }
            float x = cities.at(c_1).x - cities.at(c_2).x; //calculating number to square in next step
            float y = cities.at(c_1).y - cities.at(c_2).y;
            float dist;
            dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	        dist = sqrt(dist);

            //Making distance something else
            


            matrix.at(c_1).at(c_2) = dist;
            matrix.at(c_2).at(c_1) = dist; //Symmetric matrix
        }
    }
    distance_matrix = matrix;
}

std::vector<Point> Cities::get_cities(void){
    return cities;
}

std::vector<std::vector<float>> Cities::get_distance_matrix(void){
    return distance_matrix;
}

float Cities::total_distance(const std::vector<int> &path){
    float distance = 0;
    assert(number_of_cities == path.size() && "Path size and number of cities do not match!");
    for(size_t i=0; i<(number_of_cities-1); i++){
        distance += distance_matrix.at(path.at(i)).at(path.at(i+1));
    }
    distance += distance_matrix.at(path.at(number_of_cities-1)).at(path.at(0));
    return distance;
}

uint Cities::get_number_of_cities(void){
    return number_of_cities;
}

Point Cities::get_min_coordinates(void){
    return min_coordinates;
}

Point Cities::get_max_coordinates(void){
    return max_coordinates;
}

// void Cities::set_cities_from_vector(std::vector<Point> cities_vector){
//     number_of_cities = cities_vector.size();
//     cities = cities_vector;
//     calculate_distance_matrix();
// }


// add below - my own stuff

// double totalDistanceFromPlan(geometry_msgs::Point g1, geometry_msgs::Point g2)
// {
//     double totaldist = 0;
//     std::vector<geometry_msgs::Point> path; 
//     path = planBetweenTwoGoals(g1, g2, 50);
//     std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
//     if(waypointssize_<2) // to ensure a plan is made
//     {
//         path = planBetweenTwoGoals(g1, g2, 10);
//         //std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
//     if(waypointssize_<2) // to ensure a plan is made
//     {
//         path = planBetweenTwoGoals(g1, g2, 2);
//         //std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
//     for (size_t i = 0; i < path.size() - 1; ++i)
//     {
//         double distance = DistanceBetweenGoals(path[i], path[i + 1]);
//         totaldist = totaldist + distance;     
//         //std::cout << "total dist is: " << totaldist << std::endl;
//     } 
//     return totaldist;
// }


// std::vector<geometry_msgs::Point> planBetweenTwoGoals(geometry_msgs::Point st, geometry_msgs::Point en, int n)
// {
//     std::vector<geometry_msgs::Point> points;
//   // Create a request message for the service
//     nav_msgs::GetPlan srv;
//     srv.request.start.header.frame_id = "map";
//     srv.request.start.pose.position.x = st.x;
//     srv.request.start.pose.position.y = st.y;
//     srv.request.start.pose.orientation.w = 1.0;

//     srv.request.goal.header.frame_id = "map";
//     srv.request.goal.pose.position.x = en.x;
//     srv.request.goal.pose.position.y = en.y;
//     srv.request.goal.pose.orientation.w = 1.0;

//     // Call the service - the statement within the if() calls the service with srv
//     if (make_plan_.call(srv)) {
//         if (!srv.response.plan.poses.empty())
//         {
//             //ROS_INFO("Plan received with %ld poses", srv.response.plan.poses.size());
//             for (size_t i = 0; i < srv.response.plan.poses.size(); i += n)
//             {   
//                 points.push_back(srv.response.plan.poses[i].pose.position);
//                 double x = srv.response.plan.poses[i].pose.position.x;
//                 double y = srv.response.plan.poses[i].pose.position.y;
//                 //std::cout << "Position: (" << x << " , " << y << ")." << std::endl; 
//             }
//             // if last point is not end point / goal
//             if(!(points.back()==en))
//             {
//                 //std::cout << "Since point vector doesn't have end goal as last point, pushing it back." << std::endl; 
//                 points.push_back(en);
//                 //std::cout << "Position: (" << en.x << " , " << en.y << ")." << std::endl; 
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(5)); //25

//         } 
//         else 
//         {
//             //ROS_WARN("Received an empty plan so removing last random goal. Generating new goal...");
//         }
//     } 
//     else
//     {
//         ROS_ERROR("Failed to call service make_plan");
//     }

//     waypointssize_ = points.size();
//     //std::cout << "Number of elements in the simplified points vector is: " << waypointssize_ << std::endl;
//     return points;
// }
