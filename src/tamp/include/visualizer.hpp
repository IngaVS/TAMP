#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class Visualizer {
private:
  ros::NodeHandle nh;
  ros::Publisher stl_pub_;
  visualization_msgs::MarkerArray slt_marker_;
  ros::Publisher zlq_pub_;
  visualization_msgs::MarkerArray zlq_marker_;
  ros::Publisher soil_pub_;
  geometry_msgs::PolygonStamped soil_marker_;
  ros::Publisher build_pub_;
  geometry_msgs::PolygonStamped build_marker_;
public:
	Visualizer();
	Visualizer(ros::NodeHandle &nh_);
	~Visualizer();
	void Publish();
private:
 	void VehiclePub();
 	void ZLQPub();
 	void SoilPub();
 	void BuildPub();
 	geometry_msgs::Quaternion headingToQuaternion(double heading);

//for test
public:
	Eigen::Vector3d SendSoilPos() const;
	Eigen::Vector3d GetCupPos() const;
	Eigen::Vector3d SendCupPos() const;
	Eigen::Vector3d GetBrickPos() const;
	Eigen::Vector3d ChargePos() const;
	Eigen::Vector3d SoilRegion() const {return Eigen::Vector3d(soil_pos_x, soil_pos_y, 0.0);};
	Eigen::Vector3d BuildRegion() const {return Eigen::Vector3d(build_pos_x, build_pos_y, 0.0);};
	void setRobotNum(int num){
		robot_num_ = num;
	}
	void setPos(const std::vector<Eigen::Vector3d>& p) {
		robot_pos_ = p;
	}
	void setText(const std::vector<std::string>&  s){
		robot_text_ = s;
	}
	void getRobotHead(double h){
		robot_heading_ = h;
	}
	void setTraj(const std::vector<std::vector<Eigen::Vector3d>>& t){
		traj_ = t;
	}

	void setTask(bool meterial) {
		meterial_ = meterial;
	}
private:
  double pos_x = 9.0;
  double pos_y = 0.0;
  double pos_z = 0.0;
  double length = 5.0;
  double direction = M_PI / 3.0;
  double soil_pos_x = -9.0;
  double soil_pos_y = -3.0;
  double build_pos_x = 0.0;
  double build_pos_y = 9.0;
  double robot_heading_;
  int robot_num_;
  std::vector<Eigen::Vector3d> robot_pos_;
  std::vector<std::string> robot_text_;
  std::vector<std::vector<Eigen::Vector3d>> traj_;
  // Eigen::Vector3d robot_pos_;
  // std::string robot_text_;
  std::vector<Eigen::Vector3d> ZLQ_module_;
  // std::vector<Eigen::Vector3d> traj_;
  bool meterial_;
  bool last_meterial_;
};