#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#define ACKERMANN 	   2

using namespace std;

// ros interface
ros::Subscriber odom_sub;
ros::Publisher  mesh_pub;
visualization_msgs::Marker marker;

Eigen::Quaterniond q_mesh;
Eigen::Vector3d pos_mesh;
Eigen::Vector3d last_pos{1000000, 1000000, 1000000};
double last_yaw = 1000000;

void odomCallback(const nav_msgs::OdometryPtr &msg)
{
	Eigen::Vector3d pos{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
	Eigen::Quaterniond qyaw{msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};

	// Eigen::Quaterniond q = (q_mesh*rot_q).normalized();
	// marker.pose.position.x = pos[0];
	// marker.pose.position.y = pos[1];
	// marker.pose.position.z = pos[2];
	// marker.pose.orientation.w = q.w();
	// marker.pose.orientation.x = q.x();
	// marker.pose.orientation.y = q.y();
	// marker.pose.orientation.z = q.z();
	Eigen::Matrix3d R(qyaw);
    double yaw = atan2(R.col(0)[1],R.col(0)[0]);

	if( (pos - last_pos).norm() > 0.02 || fabs(yaw - last_yaw) > 0.02 )
	{
		last_pos = pos;
		last_yaw = yaw;

		Eigen::Quaterniond q = (qyaw * q_mesh).normalized();
		Eigen::Vector3d dp = R*pos_mesh;
		marker.pose.position.x = pos[0] - dp.x();
		marker.pose.position.y = pos[1] - dp.y();
		marker.pose.position.z = 0.0;
		marker.pose.orientation.w = q.w();
		marker.pose.orientation.x = q.x();
		marker.pose.orientation.y = q.y();
		marker.pose.orientation.z = q.z();
	}
	mesh_pub.publish(marker);
}

// main loop
int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "mesh_vis_for_realworld_node");
    ros::NodeHandle nh("~");
	
    odom_sub  = nh.subscribe("realworld_odom", 1, odomCallback, ros::TransportHints().tcpNoDelay());
	mesh_pub = nh.advertise<visualization_msgs::Marker>("mesh", 10);
	
	marker.header.frame_id = "world";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color.a = 1.0;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.scale.z = 0.001;
	Eigen::Matrix3d R_mesh;
	R_mesh << 0, 0,-1,
					 -1, 0, 0,
					  0, 1, 0;
	q_mesh = Eigen::Quaterniond{R_mesh};
	pos_mesh = Eigen::Vector3d(-0.325, -0.316, 0.0);
	marker.mesh_resource = "package://kinematics_simulator/meshes/SCOUT_MINI.STL";

	ros::spin();

    return 0;
}