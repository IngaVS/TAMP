#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include "task_info.hpp"
using namespace std;

// 定义一个结构体来存储机器人类型的信息
struct Robot {
    int TypeId;
    int Robot_num;
    double Velocity;
    double Move_power;
    double Move_temperature;
    double Cold_down_temperature;
    double Cold_down_power;
    double Max_temperature;
    std::string task_file;
};

struct TaskConfig {
    std::string name;
    double time;
    double power;
    double temperature;
    std::vector<int> pos;
    std::vector<std::string> neighbors;
};
class Config {
public:
	Config(){};
    Config(const ros::NodeHandle &nh_priv) {
        
    	// 获取机器人类型的数量
	    int robot_types;
	    std::string robot_types_param = "/RobotTypes";
	    if (nh.getParam(robot_types_param, robot_types)) {
	        ROS_INFO("Successfully retrieved parameter: %s", robot_types_param.c_str());
	        ROS_INFO("RobotTypes: %d", robot_types);
	    } else {
	        ROS_ERROR("111 Failed to get param '%s'", robot_types_param.c_str());
	        return 1;
	    }

	    // 获取机器人参数
	    std::vector<Robot> robots;
	    for (int i = 0; i < robot_types; ++i) {
	        Robot robot;
	        std::string robot_base = "/Robots/" + std::to_string(i);

	        if (!nh.getParam(robot_base + "/TypeId", robot.TypeId) ||
	            !nh.getParam(robot_base + "/Robot_num", robot.Robot_num) ||
	            !nh.getParam(robot_base + "/Velocity", robot.Velocity) ||
	            !nh.getParam(robot_base + "/Move_power", robot.Move_power) ||
	            !nh.getParam(robot_base + "/Move_temperature", robot.Move_temperature) ||
	            !nh.getParam(robot_base + "/Cold_down_temperature", robot.Cold_down_temperature) ||
	            !nh.getParam(robot_base + "/Cold_down_power", robot.Cold_down_power) ||
	            !nh.getParam(robot_base + "/Max_temperature", robot.Max_temperature) ||
	            !nh.getParam(robot_base + "/task_file", robot.task_file)) {
	            ROS_ERROR("Failed to get some params for robot %d", i);
	            continue;
	        }

	        robots.push_back(robot);
	    }






        // 获取全局参数
        std::cout<<"11111111"<<std::endl;
        nh_priv.getParam("Velocity", velocity);
        nh_priv.getParam("Move_power", move_power);
        nh_priv.getParam("Move_temperature", move_temperature);
        nh_priv.getParam("Cold_down_temperature", cold_down_temperature);
        nh_priv.getParam("Cold_down_power", cold_down_power);
        nh_priv.getParam("Max_temperature", max_temperature);
        // 获取任务列表
        if (nh_priv.getParam("Tasks", task_names)) {
            for (const std::string& task_name : task_names) {
                TaskConfig task;
                task.name = task_name;
                std::cout<<"taskname "<<task_name<<std::endl;
                // 从私有命名空间获取任务参数
                nh_priv.getParam(task_name + "/time", task.time);
                nh_priv.getParam(task_name + "/power", task.power);
                nh_priv.getParam(task_name + "/temperature", task.temperature);
                nh_priv.getParam(task_name + "/pos", task.pos);
                nh_priv.getParam(task_name + "/neighbors", task.neighbors);
                task_params.insert(make_pair(task_name, task));
            }
        } else {
            ROS_ERROR("Failed to get param 'Tasks'");
        }
    }
    double velocity;
    double move_power;
    double move_temperature;
    double cold_down_temperature;
    double cold_down_power;
    double max_temperature;
    std::vector<std::string> task_names;
    std::unordered_map<std::string, TaskConfig> task_params;
};
struct ZLQStatus{
	int cup_num;
	int brick_num;
	int build_num;
};
struct TaskNode{
	std::unordered_map<std::string, TaskConfig>::const_iterator task_ptr;
	//get result after motion planning
	std::map<double, TaskNode*> neighbors;
	std::unordered_set<string> expanded_name;
	// RobotState end_state;
	// TaskProcess task_process;
	// Trajectory charge_traj;

	// string name;
	TaskNode* parent;
	double g;
	double h;

	TaskNode(std::unordered_map<std::string, TaskConfig>::const_iterator itr){
		// name = nm;
		task_ptr = itr;
		parent = nullptr;
		expanded_name.clear();
		neighbors.clear();
		g = 0.0;
		h = 0.0;
	}
	// 计算节点的 f 值（f = g + h）
    double f() const {
        return g + h;
    }
};

// // 自定义优先队列的比较函数
// struct CompareNode {
//     bool operator()(const RobotTask* n1, const RobotTask* n2) const {
//         return n1->f() > n2->f();
//     }
// };


class RobotTask {
public:
	RobotTask();
	RobotTask(ros::NodeHandle &nh);
	~RobotTask();
	void Run();
	void Init(const Config& config);
private:
	double g_func(const TaskConfig& st, const TaskConfig& et);
	double h_func(const TaskConfig& et);
	ros::NodeHandle nh_;
	Config config_;
	vector<TaskNode> task_seq_;
	TaskNode* cur_node_;

};