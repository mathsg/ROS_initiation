#include <algorithm>
#include <math.h>
#include <vector>
#include <ros/ros.h>

//The messages
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <simu_msgs/vectorList.h>

float control_frequency = 10.0;
double pi = 3.14;

class MapVisualization
{
  public:
	ros::NodeHandle n;
  //Subscribers
	ros::Subscriber map_point_sub;
  //Publishers
  ros::Publisher map_point_cloud_pub;

	MapVisualization()
	{
		// Parameters
		// n.param<double>("/robot_visualization/robot_radius", robot_radius, 0.02);

    //The different subscribes
		map_point_sub = n.subscribe("/the_map/the_map_points", 1, &MapVisualization::map_points_callback, this);

    //The different publishers
		map_point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/visualization/map_visualization/the_map_cloud", 1);

    //The different initialisations
    the_map_cloud.header.frame_id = "map";

	}

  void publish_the_maps()
  {
    //Update the frame of the robot
    map_point_cloud_pub.publish( the_map_cloud );

  }


  private:

    void map_points_callback(const simu_msgs::vectorList::ConstPtr &msg)
  	{
  		the_map_points = *msg;
      map_cloud_update();
  	}

    void map_cloud_update()
    {
      //Declare the point32
      geometry_msgs::Point32 one_point;

      //Reinitialize the whole point cloud
      the_map_cloud.points.clear();

      for(int i=0; i<the_map_points.number; i++)
      {
        one_point.x = the_map_points.points[i].x;
        one_point.y = the_map_points.points[i].y;
        one_point.z = the_map_points.points[i].z;

        the_map_cloud.points.push_back( one_point );
      }

    }

    //The subscribers_msgs
    simu_msgs::vectorList the_map_points;
    //The publishers
    sensor_msgs::PointCloud the_map_cloud;


    //The main parameters
    double robot_radius, wheels_radius;

    //The inner variables
    ros::Time t;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map visualization");

	MapVisualization map_visualization_;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("Visualization : map visualization running");

	while (map_visualization_.n.ok())
	{
    map_visualization_.publish_the_maps();
		ros::spinOnce();
		loop_rate.sleep();
	}

  ros::spin();

	return 0;
}
