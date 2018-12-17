#include <algorithm>
#include <math.h>
#include <vector>
#include <ros/ros.h>

//The messages
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <simu_msgs/RobotPose.h>

float control_frequency = 10.0;
double pi = 3.14;

class RobotVisualization
{
  public:
	ros::NodeHandle n;
  //Subscribers
	ros::Subscriber pose_sub;
  //Publishers
  ros::Publisher robot_corpse_visu_pub, robot_flat_visu_pub, robot_gripper_visu_pub;
  //Transformers
  tf::TransformBroadcaster robot_br;

	RobotVisualization()
	{
		// Parameters
		n.param<double>("/robot_visualization/robot_radius", robot_radius, 0.02);
    n.param<double>("/robot_visualization/wheels_radius", wheels_radius, 0.02);

    //The different subscribes
		pose_sub = n.subscribe("/kinematics/robot_pose", 1, &RobotVisualization::pose_callback, this);

    //The different publishers
		robot_corpse_visu_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/robot_visualization/the_robot", 1);
    robot_flat_visu_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/robot_visualization/the_flat_robot", 1);
    robot_gripper_visu_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization/robot_visualization/the_robot_gripper", 1);

    //Initialisation
    markers_initialisation();
	}

  void updateRobot()
  {
    //Update the frame of the robot
    tf::Transform robot_transform;
    t = ros::Time::now();

    float x_pos = the_robot_pose.pose.linear.x;
    float y_pos = the_robot_pose.pose.linear.y;
    float z_pos = the_robot_pose.pose.linear.z;
    float x_angle = the_robot_pose.pose.angular.x;
    float y_angle = the_robot_pose.pose.angular.y;
    float z_angle = the_robot_pose.pose.angular.z;

    robot_transform.setOrigin( tf::Vector3(x_pos, y_pos, z_pos) );
    tf::Quaternion q;
    q.setRPY(x_angle, y_angle, z_angle);
    robot_transform.setRotation(q);
    robot_br.sendTransform(tf::StampedTransform(robot_transform, t, "map", "robot"));

    //Update the frame of the gripper


    //Publish the markers
    robot_corpse_visu_pub.publish( the_main_corpse );
    robot_flat_visu_pub.publish( the_flat_corpse );
    robot_gripper_visu_pub.publish( the_gripper );

  }


  private:

    void pose_callback(const simu_msgs::RobotPose::ConstPtr &msg)
  	{
  		the_robot_pose = *msg;
  	}

    void markers_initialisation()
    {
      //Initalize the corpse of the robot
      corpse_initialisation();

      //Initialize the flat corpse
      flat_robot_initialisation();

      //Initialise the gripper of the robot
      gripper_initialisation();
    }

    void corpse_initialisation()
    {
      visualization_msgs::Marker one_corpse_part;
      tf::Quaternion q;

      //Common parts
      one_corpse_part.header.frame_id = "robot";
      one_corpse_part.header.stamp = ros::Time();
      one_corpse_part.ns = "world";

      //Create the corpse of the robot
      one_corpse_part.id = 0;
      one_corpse_part.type = visualization_msgs::Marker::CYLINDER;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.0;
      one_corpse_part.pose.position.y = 0.0;
      one_corpse_part.pose.position.z = wheels_radius;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 2 * robot_radius;
      one_corpse_part.scale.y = 2 * robot_radius;
      one_corpse_part.scale.z = 0.4 * robot_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.9;
      one_corpse_part.color.g = 0.9;
      one_corpse_part.color.b = 0.9;

      the_main_corpse.markers.push_back( one_corpse_part );

      //Create the wheels of the robot
      one_corpse_part.id++;
      q.setRPY(pi/2, 0, 0);

      one_corpse_part.type = visualization_msgs::Marker::CYLINDER;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.0;
      one_corpse_part.pose.position.y = robot_radius;
      one_corpse_part.pose.position.z = wheels_radius;
      one_corpse_part.pose.orientation.x = q[0];
      one_corpse_part.pose.orientation.y = q[1];
      one_corpse_part.pose.orientation.z = q[2];
      one_corpse_part.pose.orientation.w = q[3];
      one_corpse_part.scale.x = 2 * wheels_radius;
      one_corpse_part.scale.y = 2 * wheels_radius;
      one_corpse_part.scale.z = 0.4 * wheels_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.0;
      one_corpse_part.color.g = 1.0;
      one_corpse_part.color.b = 0.2;
      the_main_corpse.markers.push_back( one_corpse_part );

      one_corpse_part.id++;
      one_corpse_part.pose.position.y = - robot_radius;
      the_main_corpse.markers.push_back( one_corpse_part );

      //Create the head
      one_corpse_part.id++;
      one_corpse_part.type = visualization_msgs::Marker::CUBE;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.6 * robot_radius;
      one_corpse_part.pose.position.y = 0.0;
      one_corpse_part.pose.position.z = wheels_radius + 0.8 * wheels_radius;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 0.6 * robot_radius;
      one_corpse_part.scale.y = 1.5 * robot_radius;
      one_corpse_part.scale.z = 0.4 * robot_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.1;
      one_corpse_part.color.g = 0.1;
      one_corpse_part.color.b = 0.1;
      the_main_corpse.markers.push_back( one_corpse_part );

      //The robot eyes
      one_corpse_part.id++;
      q.setRPY(0, pi/2, 0);

      one_corpse_part.type = visualization_msgs::Marker::CYLINDER;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.6 * robot_radius + 0.3 * robot_radius;
      one_corpse_part.pose.position.y = 0.5 * robot_radius;
      one_corpse_part.pose.position.z = wheels_radius + 0.8 * wheels_radius;
      one_corpse_part.pose.orientation.x = q[0];
      one_corpse_part.pose.orientation.y = q[1];
      one_corpse_part.pose.orientation.z = q[2];
      one_corpse_part.pose.orientation.w = q[3];
      one_corpse_part.scale.x = 0.2 * robot_radius;
      one_corpse_part.scale.y = 0.2 * robot_radius;
      one_corpse_part.scale.z = 0.2 * robot_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.0;
      one_corpse_part.color.g = 0.0;
      one_corpse_part.color.b = 0.5;
      the_main_corpse.markers.push_back( one_corpse_part );

      one_corpse_part.id++;
      one_corpse_part.pose.position.y = - one_corpse_part.pose.position.y;
      the_main_corpse.markers.push_back( one_corpse_part );

    }

    void flat_robot_initialisation()
    {
      visualization_msgs::Marker one_corpse_part;
      tf::Quaternion q;

      //Common parts
      one_corpse_part.header.frame_id = "robot";
      one_corpse_part.header.stamp = ros::Time();
      one_corpse_part.ns = "world";

      //Create the corpse (circle)
      one_corpse_part.id = 0;
      one_corpse_part.type = visualization_msgs::Marker::CYLINDER;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.0;
      one_corpse_part.pose.position.y = 0.0;
      one_corpse_part.pose.position.z = 0.005;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 2 * robot_radius;
      one_corpse_part.scale.y = 2 * robot_radius;
      one_corpse_part.scale.z = 0.01;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.9;
      one_corpse_part.color.g = 0.9;
      one_corpse_part.color.b = 0.9;

      the_flat_corpse.markers.push_back( one_corpse_part );

      //Create the wheels (squares)
      one_corpse_part.id++;
      one_corpse_part.type = visualization_msgs::Marker::CUBE;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.0;
      one_corpse_part.pose.position.y = robot_radius;
      one_corpse_part.pose.position.z = 0.006;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 2 * wheels_radius;
      one_corpse_part.scale.y = 0.4 * wheels_radius;
      one_corpse_part.scale.z = 0.012;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.0;
      one_corpse_part.color.g = 1.0;
      one_corpse_part.color.b = 0.2;
      the_flat_corpse.markers.push_back( one_corpse_part );

      one_corpse_part.id++;
      one_corpse_part.pose.position.y = - robot_radius;
      the_flat_corpse.markers.push_back( one_corpse_part );

      //Create the center point (sphere)
      one_corpse_part.id++;
      one_corpse_part.type = visualization_msgs::Marker::SPHERE;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = 0.0;
      one_corpse_part.pose.position.y = 0.0;
      one_corpse_part.pose.position.z = 0.01;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 0.3 * robot_radius;
      one_corpse_part.scale.y = 0.3 * robot_radius;
      one_corpse_part.scale.z = 0.3 * robot_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 1.0;
      one_corpse_part.color.g = 0.2;
      one_corpse_part.color.b = 0.2;
      the_flat_corpse.markers.push_back( one_corpse_part );

      //Create the direction (arrow)
      one_corpse_part.id++;
      one_corpse_part.type = visualization_msgs::Marker::ARROW;
      one_corpse_part.action = visualization_msgs::Marker::ADD;
      one_corpse_part.pose.position.x = - 0.8 * robot_radius;
      one_corpse_part.pose.position.y = 0.0;
      one_corpse_part.pose.position.z = 0.01;
      one_corpse_part.pose.orientation.x = 0.0;
      one_corpse_part.pose.orientation.y = 0.0;
      one_corpse_part.pose.orientation.z = 0.0;
      one_corpse_part.pose.orientation.w = 1.0;
      one_corpse_part.scale.x = 1.8 * robot_radius;
      one_corpse_part.scale.y = 0.2 * robot_radius;
      one_corpse_part.scale.z = 0.1 * robot_radius;
      one_corpse_part.color.a = 1.0; // Don't forget to set the alpha!
      one_corpse_part.color.r = 0.0;
      one_corpse_part.color.g = 0.0;
      one_corpse_part.color.b = 0.0;
      the_flat_corpse.markers.push_back( one_corpse_part );
    }

    void gripper_initialisation()
    {

    }


    //The subscribers_msgs
    simu_msgs::RobotPose the_robot_pose;
    //The publishers_msgs
    visualization_msgs::MarkerArray the_main_corpse;
    visualization_msgs::MarkerArray the_flat_corpse;
    visualization_msgs::MarkerArray the_gripper;

    //The main parameters
    double robot_radius, wheels_radius;

    //The inner variables
    ros::Time t;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot visualization");

	RobotVisualization robot_visualization_;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("Visualization : robot visualization running");

	while (robot_visualization_.n.ok())
	{
    robot_visualization_.updateRobot();
		ros::spinOnce();
		loop_rate.sleep();
	}

  ros::spin();

	return 0;
}
