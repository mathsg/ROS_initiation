// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>

//The messages
#include <geometry_msgs/Vector3.h>
#include <simu_msgs/vectorList.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>



// std includes
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "own_map");
    ros::NodeHandle n("~");
    ros::Rate r(10);

    float discretisation_step;

    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");
    n.param<float>("/own_map/discretization_step", discretisation_step, 0.01);

    // string obss_file;
    // n.param<std::string>("/own_map/obss_file", obss_file, "obss.txt");

//    n.param<string>("map_frame", _map_frame, "/map");
//    n.param<string>("map_topic", _map_topic, "/maze_map");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);
    ROS_INFO_STREAM("The maze map will be published in frame " << _map_frame);
    ROS_INFO_STREAM("The maze map will be published on topic " << _map_topic);

    ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    ros::Publisher points_coordinates_pub = n.advertise<simu_msgs::vectorList>("/the_map/the_map_points", 1);
    std::vector<geometry_msgs::Vector3> wall_points;

    int length = 0;


    string line;

    int wall_id = 0;
    while (getline(map_fs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
        }

        geometry_msgs::Vector3 point;
        point.x = x1;
        point.y = y1;
        point.z = 0.0;
        wall_points.push_back(point);

        point.x = x2;
        point.y = y2;
        point.z = 0.0;
        wall_points.push_back(point);

        //Discretized map
        int N_step = floor( sqrt(pow(x1-x2,2) + pow(y1-y2,2))/discretisation_step ) + 1;
        float x_step = (x2-x1)/N_step;
        float y_step = (y2-y1)/N_step;
        for(int i=0; i<N_step + 1; i++)
        {
          point.x = x1 + i*x_step;
          point.y = y1 + i*y_step;
          point.z = 0;
          wall_points.push_back(point);
          length ++;
        }
    }

    simu_msgs::vectorList map_points;
    map_points.number = wall_points.size();
    map_points.points = wall_points;

    // Main loop.
    while (n.ok())
    {
        points_coordinates_pub.publish( map_points );
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
