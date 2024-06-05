#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <map>
#include "motion_planning.hpp"
#include "config.hpp"
#include "data_pool.hpp"
using namespace std;

struct TaskNode{
	std::string task_name;
	std::multimap<double, std::shared_ptr<TaskNode>> neighbors;
	std::unordered_set<string> expanded_name;
	std::shared_ptr<TaskNode> parent;
	double g;
	double h;

	//get result after motion planning
	RobotState end_state;
	TaskProcess task_process;
	Trajectory charge_traj;

	TaskNode(const string name){
		task_name = name;
		parent = nullptr;
		expanded_name.clear();
		neighbors.clear();
		g = 0.0;
		h = 0.0;
		end_state = RobotState(0.0, 1.0, 15);
		task_process = TaskProcess();
	}
	// 计算节点的 f 值（f = g + h）
    double f() const {
        return g + h;
    }
};

class RobotTask {
public:
	RobotTask();
	RobotTask(ros::NodeHandle &nh);
	~RobotTask();
	void Run();
	void Init(const TaskGraph& config, const Robot& robot);
	void GetInfo();
	void SetStartTime(const double st){start_time_ = st;};
	void SetMap(const VoxelMap* vm){voxel_map_ = vm;}
	string getText() const {return robot_text_;}
	Eigen::Vector3d getPos()const {return pos_;}
	vector<Eigen::Vector3d> getTraj()const {return traj_;}
	bool getMeterial() const {return vis_meterial_;}

private:
	double g_func(const TaskConfig& st, const TaskConfig& et);
	double h_func(const TaskConfig& et);
	void FindPubTask(double t);
	bool PreconValid(const vector<string>& precons, 
							const vector<int>& sp,
							const vector<int>& ep,
							const RobotState& cur_state);

	bool UpdateTask(const RobotState& begin_task_state, 
					const TaskConfig& cur_task_param,
					const TaskConfig& next_task_param, 
					std::shared_ptr<TaskNode> next_node);
	void UpdateState(const TaskConfig& next_task_param, std::shared_ptr<TaskNode> next_node);

	RobotState CalculateState(const Trajectory& traj, const RobotState& begin_state, 
														vector<RobotState>& robot_state_vec);
	RobotState CalculateOperationalState(const  TaskConfig& next_task_param, const RobotState& begin_state, 
												vector<RobotState>& robot_state_vec);

	ros::NodeHandle nh_;
	TaskGraph config_;
	Robot robot_;
	DataPool* data_pool_;
	vector<TaskNode> task_vec_;
	double start_time_;
	std::shared_ptr<TaskNode> cur_node_;
	int cycle_;
	
	const VoxelMap* voxel_map_;

	size_t find_task_idx_;
	Eigen::Vector3d pos_;
	string robot_text_;
	bool vis_meterial_;
	vector<Eigen::Vector3d> traj_;
	ros::Publisher pub_;
};