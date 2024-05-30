#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "task_info.hpp"
using namespace std;
struct Config
{
	double max_velocity;
	double max_temperature;
	double move_power;
	double move_temperature;
	double cold_down_temperature;
	double cold_down_power;
	double get_soil_time;
	double get_soil_power;
	double get_soil_temperature;
	
	double fill_soil_time;
	double fill_soil_power;
	double fill_soil_temperature;
	
	double get_cup_time;
	double get_cup_power;
	double get_cup_temperature;
	double fill_cup_time;
	double fill_cup_power;
	double fill_cup_temperature;
	double get_brick_time;
	double get_brick_power;
	double get_brick_temperature;
	double build_brick_time;
	double build_brick_power;
	double build_brick_temperature;

	double charge_power_speed;

	Eigen::Vector3d get_soil_pos;
	Eigen::Vector3d fill_soil_pos;
	Eigen::Vector3d get_cup_pos;
	Eigen::Vector3d fill_cup_pos;
	Eigen::Vector3d get_brick_pos;
	Eigen::Vector3d charge_pos;
	Eigen::Vector3d build_brick_pos;
  Config(const ros::NodeHandle &nh_priv)
  {
    nh_priv.getParam("Velocity", max_velocity);
    nh_priv.getParam("Move_power", move_power);
    nh_priv.getParam("Move_temperature", move_temperature);
    nh_priv.getParam("Max_temperature", max_temperature);
    nh_priv.getParam("Cold_down_temperature", cold_down_temperature);
    nh_priv.getParam("Cold_down_power", cold_down_power);

    nh_priv.getParam("Get_soil_time", get_soil_time);
    nh_priv.getParam("Get_soil_time", get_soil_power);
    nh_priv.getParam("Get_soil_time", get_soil_temperature);

    nh_priv.getParam("Fill_soil_time", fill_soil_time);
    nh_priv.getParam("Fill_soil_power", fill_soil_power);
    nh_priv.getParam("Fill_soil_temperature", fill_soil_temperature);

    nh_priv.getParam("Get_cup_time", get_cup_time);
    nh_priv.getParam("Get_cup_power", get_cup_power);
    nh_priv.getParam("Get_cup_temperature", get_cup_temperature);

    nh_priv.getParam("Fill_cup_time", fill_cup_time);
    nh_priv.getParam("Fill_cup_power", fill_cup_power);
    nh_priv.getParam("Fill_cup_temperature", fill_cup_temperature);

    nh_priv.getParam("Get_brick_time", get_brick_time);
    nh_priv.getParam("Get_brick_power", get_brick_power);
    nh_priv.getParam("Get_brick_temperature", get_brick_temperature);

    nh_priv.getParam("Build_brick_time", build_brick_time);
    nh_priv.getParam("Build_brick_power", build_brick_power);
    nh_priv.getParam("Build_brick_temperature", build_brick_temperature);
    nh_priv.getParam("Charge_power_speed", charge_power_speed);
  }
};

class TaskPlanning{
public:
	TaskPlanning();
	TaskPlanning(ros::NodeHandle &nh);
	~TaskPlanning();
	void Run();
	void Init(Config& config);
	void GetInfo();
	string getText() const {return robot_text_;}
	Eigen::Vector3d getPos()const {return pos_;}
	vector<Eigen::Vector3d> getTraj()const {return traj_;}
	bool getMeterial() const {return vis_meterial_;}
private:
	void UpdateTask(const RobotTask& next_task);
	RobotState CalculateState(const Trajectory& traj, const RobotState& begin_state, 
														vector<RobotState>& robot_state_vec);
	RobotState CalculateOperationalState(const RobotTask& next_task, const RobotState& begin_state,
														vector<RobotState>& robot_state_vec);
	void FindPubTask(double t);
	void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
private:
	ros::NodeHandle nh_;
	RobotInfo robot_info_;
	RobotParam robot_param_;
	RobotTask robot_current_task_;
	RobotTask get_soil_task_;
	RobotTask fill_soil_task_;
	RobotTask get_cup_task_;
	RobotTask fill_cup_task_;
	RobotTask get_brick_task_;
	RobotTask build_brick_task_;
	RobotTask charge_task_;
	RobotTask* after_charged_task_;
	ZLQTask sift_soil_task_;
	ZLQTask make_brick_task_;
	ZLQTask robot_charge_task_;

	VoxelMap voxel_map_;
	bool map_initialized_;
	ros::Subscriber map_sub_;

	std::vector<RobotTask> task_vec_;
	size_t find_task_idx_;
	double start_time_;

	//output
	Eigen::Vector3d pos_;
	string robot_text_;
	bool vis_meterial_;
	vector<Eigen::Vector3d> traj_;
	ros::Publisher pub_;
 

};