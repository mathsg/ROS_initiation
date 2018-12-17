#include <algorithm>
#include <math.h>
#include <vector>
#include "ros/ros.h"

//The messages
#include <visualization_msgs/MarkerArray.h>

float control_frequency = 10.0;

class RobotVisualization
{
  public:
	ros::NodeHandle n;
	ros::Subscriber pose_sub;
  ros::Publisher robot_visu_pub.

	RobotVisualization()
	{
		// Parameters
		// n.param<double>("/heuristic_grids_server/grid_square_size", grid_square_size, 0.02);

    //The different subscribes
		pose_sub = n.subscribe("/kinematics/robot_pose", 1, &RobotVisualization::pose_callback, this);

    //The different publishers
		robot_visu_pub = n.advertise<visualization_msgs::marker_array>("/visualization/robot_visualization/the_robot", 1);

	}




  private:

    void pose_callback(const pose_msgs::RobotPose::ConstPtr &msg)
  	{
  		the_robot_pose = *msg;
  	}

    simu_msgs::RobotPose the_robot_pose;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot visualization");

	RobotVisualization robot_visualization_;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("Heuristic grids server running");

	while (robot_visualization_.n.ok())
	{
    robot_visualization_.updateRobot();
		ros::spinOnce();
		loop_rate.sleep();
	}

  ros::spin();

	return 0;
}
