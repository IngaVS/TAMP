#pragma once
#include <Eigen/Eigen>
#include <string>
#include "motion_planning.hpp"
using namespace std;
struct TaskParam{
	double spend_time;
	double spend_power;
	double temperature;
	TaskParam(){
		spend_time = 0.0;
		spend_power = 0.0;
		temperature = 0.0;
	}
};

enum TaskType{
	NONE = 0,
	GET_SOIL,
	FILL_SOIL,
	GET_CUP,
	FILL_CUP,
	GET_BRICK,
	BUILD_BRICK,
	CHARGE
};


struct ZLQTask{
	string name;
	bool operating;
	bool has_meterial;
	int counter;
	TaskParam param;
	ZLQTask(){
		name = "empty";
		operating = false;
		has_meterial = false;
		param = TaskParam();
		counter = 0;
	}
	void resetCounter(){counter = 0;}
	void beginOperate(){operating = true;}
	bool isOperating(){return operating;}
	void countFunction(){
		if(counter > 3){
			has_meterial = true;
			operating = false;
			// std::cout<<"has_meterial "<<endl;
		}else{
			counter++;
			has_meterial = false;
		}
	}
};

enum ProcessType{
	WAIT = 0,
	MOVING,
	OPERATING
};



struct RobotState {
	ProcessType process_type;
	double time;			//timeclock
	double power;
	double temperature;
	RobotState(){
		process_type = ProcessType::WAIT;
		time = 0.0;
		power = 0.0;
		temperature = 0.0;
	}
	RobotState(double t, double p, double tem){
		process_type = ProcessType::WAIT;
		time = t;
		power = p;
		temperature = tem;
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

struct RobotTask{
	string name;
	TaskType task_type;
	Eigen::Vector3d pos;
	TaskParam param;

	//get result after motion planning
	RobotState end_state;
	TaskProcess task_process;
	Trajectory charge_traj;
	RobotTask(){
		name = "robot empty";
		task_type = TaskType::NONE;
		pos = Eigen::Vector3d(0.0, 0.0, 0.0);
		param = TaskParam();
		// start_state = RobotState();
		end_state = RobotState();
		task_process = TaskProcess();
		charge_traj = Trajectory();
	}

	void getInput(const RobotTask& cp){
		name = cp.name;
		task_type = cp.task_type;
		pos = cp.pos;
		param = cp.param;
	}

};

struct RobotParam{
	double max_velocity;
	double max_temperature;
	double cold_down_temperature;
	double cold_down_power;
	double move_power;
	double move_temperature;
	RobotParam() {
		max_velocity = 0.0;
		move_power = 0.0;
		move_temperature = 0.0;
		max_temperature = 0.0;
		cold_down_temperature = 0.0;
		cold_down_power = 0.0;
	}
};

struct RobotInfo{
	double power;
	double temperature;
	Eigen::Vector3d pos;
	// RobotTask current_task;
	TaskType task_type;
	ProcessType process_type;
	bool task_done;
	RobotInfo() {
		power = 0.0;
		temperature = 0.0;
		pos = Eigen::Vector3d(0.0, 0.0, 0.0);
		// current_task = RobotTask();
		task_type = TaskType::NONE;
		process_type = ProcessType::WAIT;
		task_done = false;
	}
};
