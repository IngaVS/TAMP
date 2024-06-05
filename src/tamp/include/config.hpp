#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <Eigen/Eigen>
using namespace std;

// 定义一个结构体来存储机器人类型的信息
struct Robot {
    int robot_num;
    double velocity;
    double move_power;
    double move_temperature;
    double cold_down_temperature;
    double cold_down_power;
    double max_temperature;
    std::string task_file;
};
struct TaskConfig {
    std::string name;
    double time;
    double power;
    double temperature;
    std::vector<int> pos;
    std::vector<std::string> neighbors;
    vector<std::string> precons;
};
struct TaskGraph{
    std::string name;
    std::unordered_set<std::string> obj_names;
    std::vector<std::string> task_names;
    std::unordered_map<std::string, TaskConfig> task_params;
};
class Config {
public:
    std::vector<Robot> robots;
    std::unordered_map<std::string, TaskGraph> task_graphs;
public:
	Config(){};
    Config(const ros::NodeHandle &nh) {
	    // 获取机器人类型的数量
	    std::vector<std::string> robot_types;
	    std::string robot_types_param = "/task_motion_planning_node/RobotTypes";
	    if (nh.getParam(robot_types_param, robot_types)) {
	        ROS_INFO("Successfully retrieved parameter: %s", robot_types_param.c_str());
	    } else {
	        ROS_ERROR("Failed to get param '%s'", robot_types_param.c_str());
	    }
	    // 获取机器人参数
	    for (size_t i = 0; i < robot_types.size(); ++i) {
	        Robot robot;
	        std::string robot_base = "/task_motion_planning_node/" + robot_types[i];
	        if (!nh.getParam(robot_base + "/robot_num", robot.robot_num) ||
	            !nh.getParam(robot_base + "/velocity", robot.velocity) ||
	            !nh.getParam(robot_base + "/move_power", robot.move_power) ||
	            !nh.getParam(robot_base + "/move_temperature", robot.move_temperature) ||
	            !nh.getParam(robot_base + "/cold_down_temperature", robot.cold_down_temperature) ||
	            !nh.getParam(robot_base + "/cold_down_power", robot.cold_down_power) ||
	            !nh.getParam(robot_base + "/max_temperature", robot.max_temperature) ||
	            !nh.getParam(robot_base + "/task_file", robot.task_file)) {
	            ROS_ERROR("Failed to get some params for robot %d", int(i));
	            continue;
	        }
	        robots.push_back(robot);
	        if(!task_graphs.count(robot.task_file)) {
	            TaskGraph graph;
	            std::string task_base = "/task_motion_planning_node/" + robot.task_file;
	            graph.name = robot.task_file;
	            if(nh.getParam(task_base + "/task_names", graph.task_names)) {
	                for (const std::string& task_name : graph.task_names) {
	                    TaskConfig task;
	                    task.name = task_name;
	                    std::cout<<task_base + "/"+ task_name + "/precon"<<std::endl;
	                    // 从私有命名空间获取任务参数
	                   if( !nh.getParam(task_base + "/"+ task_name + "/time", task.time) ||
	                    !nh.getParam(task_base + "/"+ task_name + "/power", task.power) ||
	                    !nh.getParam(task_base + "/"+ task_name + "/temperature", task.temperature) ||
	                    !nh.getParam(task_base + "/"+ task_name + "/pos", task.pos) ||
	                    !nh.getParam(task_base + "/"+ task_name + "/neighbors", task.neighbors) ||
	                    !nh.getParam(task_base + "/"+ task_name + "/precon", task.precons)){
	                   		std::cout<<"ERROR: "<<task_name<<std::endl;
	                   }else{
	                    graph.task_params.insert(make_pair(task_name, task));
	                   }
	                }
	                task_graphs.insert(make_pair(graph.name, graph));
	            } else {
	                ROS_ERROR("Failed to get param 'task_names'");
	            }
	        }
	    }
	}

};

enum ProcessType{
	WAIT = 0,
	MOVING,
	OPERATING
};

enum RobotMaterial{
	EMPTY = 0,
	SOIL = 1,
	CUP = 2,
	BRICK = 3
};

struct RobotState {
	ProcessType process_type;
	double time;			//timeclock
	double power;
	double temperature;
	RobotMaterial material_type;
	RobotState(){
		process_type = ProcessType::WAIT;
		time = 0.0;
		power = 0.0;
		temperature = 0.0;
		material_type = RobotMaterial::EMPTY;
	}
	RobotState(double t, double p, double tem){
		process_type = ProcessType::WAIT;
		time = t;
		power = p;
		temperature = tem;
		material_type = RobotMaterial::EMPTY;
	}
};

struct TaskProcess{
	vector<RobotState> process;
	Trajectory move_traj;
	TaskProcess(){
		process.clear();
		move_traj = Trajectory();
	}
};


