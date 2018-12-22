#include <algorithm>
#include <math.h>
#include <vector>
#include <ros/ros.h>

//The messages
#include <visualization_msgs/MarkerArray.h>

float control_frequency = 10.0;

class HeuristicGrid
{
  public:
	ros::NodeHandle n;
	ros::Subscriber pose_sub;
  ros::Publisher robot_visu_pub.

	HeuristicGrid()
	{
		// Parameters
		n.param<double>("/heuristic_grids_server/grid_cell_size", grid_cell_size, 0.02);

    //The different subscribes
		pose_sub = n.subscribe("/kinematics/robot_pose", 1, &RobotVisualization::pose_callback, this);

    //The different publishers
		robot_visu_pub = n.advertise<visualization_msgs::marker_array>("/visualization/robot_visualization/the_robot", 1);

	}

  void create_the_occupancy_grid()
  {
    
  }



  private:

    double grid_cell_size;

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
