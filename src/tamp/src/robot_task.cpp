#include "robot_task.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
RobotTask::RobotTask(){

}

RobotTask::RobotTask(ros::NodeHandle &nh){
	start_time_ = 0.0;
	vis_meterial_ = false;
	task_vec_.clear();
	data_pool_ = &DataPool::Instance();
	find_task_idx_ = 0;
	robot_text_ = "";
	traj_.clear();
	pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
}

RobotTask::~RobotTask(){
}

void RobotTask::Init(const TaskGraph& config, const Robot& robot){
	config_ = config;
	robot_ = robot;
	cycle_ = 0;
	if(config_.task_names.empty()){
	    std::cerr << "Task_names empty" << std::endl;
	    std::abort(); // Abnormally terminate the program
	}
	cur_node_ = std::make_shared<TaskNode>(config_.task_names[0]);
}
void RobotTask::Run(){
	if(cycle_ < 12 && cur_node_){
		auto ptr = config_.task_params.find(cur_node_->task_name);
		if(ptr == config_.task_params.end()){
		    std::cerr << "Task_params aren't match name" << std::endl;
		    std::abort(); // Abnormally terminate the program
		}
		if(cur_node_->parent != nullptr){
			cur_node_->parent->expanded_name.insert(ptr->first);
		}
		for(string neighbor_name : ptr->second.neighbors) {
			std::cout<<"neibor name "<<neighbor_name<<std::endl;
			if(cur_node_->expanded_name.count(neighbor_name) ){
				std::cout<<"Has expanded"<<std::endl;
				continue;
			} 
			auto neighbor_param_ptr = config_.task_params.find(neighbor_name);
			if(neighbor_param_ptr == config_.task_params.end()){
				std::cout<<"No neighbor param: "<<neighbor_name<<std::endl;
				continue;
			} 

			if(neighbor_param_ptr != config_.task_params.end()) {
				shared_ptr<TaskNode> new_node = std::make_shared<TaskNode>(neighbor_name);
				auto neighbor_param = neighbor_param_ptr->second;
				if(!PreconValid(neighbor_param.precons, ptr->second.pos, 
								neighbor_param.pos, cur_node_->end_state)){
					std::cout<<"Precon isn't valid: "<<neighbor_name<<std::endl;
					continue;
				}
				if(!UpdateTask(cur_node_->end_state, ptr->second, neighbor_param, new_node)){
					std::cout<<"power isn't valid: "<<neighbor_name<<std::endl;
					continue;
				}
				std::cout<<"insert: "<<neighbor_name<<std::endl;
				new_node->parent = cur_node_;
				new_node->g =  g_func(ptr->second, neighbor_param);
				new_node->h = h_func(neighbor_param);
				cur_node_->neighbors.insert(make_pair(new_node->f(), new_node));
			}
		}
		if(cur_node_->neighbors.empty()){
			std::cout<<"neighbors is empty"<<std::endl;
			cur_node_ = cur_node_->parent;
		}else{
			cur_node_ = prev(cur_node_->neighbors.end())->second;
		}

	}
	while(cycle_ >= 12 && cur_node_){
		task_vec_.push_back(*cur_node_);
		std::cout<<"robot: "<<robot_.robot_num<<"task "<<cur_node_->task_name<<std::endl;
		cur_node_ = cur_node_->parent;
	}
	if(cycle_ == 12){
		std::reverse(task_vec_.begin(), task_vec_.end());
	}
	cycle_++;
}

double RobotTask::g_func(const TaskConfig& st, const TaskConfig& et){
	if(et.name == "charge"){
		return 0.0;
	}
	return 1.0;
}

double RobotTask::h_func(const TaskConfig& et){
	return 1.0;
}

bool RobotTask::UpdateTask(const RobotState& begin_task_state, 
							const TaskConfig& cur_task_param,
							const TaskConfig& next_task_param, 
							std::shared_ptr<TaskNode> next_node){
	vector<RobotState> next_task_process_vec;
	TaskConfig charge_task;
	if(config_.task_params.count("charge")){
		charge_task = config_.task_params.find("charge")->second;
	}else{
		std::cout<<"no charge task"<<std::endl;
		return false;
	}
	Trajectory traj_cur_nxt(cur_task_param.pos, next_task_param.pos, voxel_map_, robot_.velocity);
	RobotState after_move_state = CalculateState(traj_cur_nxt, begin_task_state, next_task_process_vec);
	RobotState after_operate_state = CalculateOperationalState(next_task_param, after_move_state, next_task_process_vec);
	vector<RobotState> tmp_vec;
	Trajectory traj_nxt_chg(next_task_param.pos, charge_task.pos, voxel_map_, robot_.velocity);
	RobotState after_mvcharge_state = CalculateState(traj_nxt_chg, after_operate_state, tmp_vec);
	if(after_mvcharge_state.power > 0.3){
		next_node->task_process.process = next_task_process_vec;
		next_node->end_state = after_operate_state;
		next_node->charge_traj = traj_nxt_chg;
		next_node->task_process.move_traj = traj_cur_nxt;
		UpdateState(next_task_param, next_node);
		return true;
	}
	return false;
}

RobotState RobotTask::CalculateState(const Trajectory& traj, const RobotState& begin_state, 
									vector<RobotState>& robot_state_vec) {
	RobotState out_state = begin_state;
	double need_temperature = begin_state.temperature + traj.totalTime() * robot_.move_temperature;
	if(need_temperature > robot_.max_temperature){
		//begin cold down process
		RobotState tmp_state = begin_state;
		tmp_state.process_type = ProcessType::WAIT;
		robot_state_vec.push_back(tmp_state);
		
		//begin move process
		double wait_time = (need_temperature - robot_.max_temperature) / fabs(robot_.cold_down_temperature);
		tmp_state.process_type = ProcessType::MOVING;
		tmp_state.temperature = robot_.max_temperature - traj.totalTime() * robot_.move_temperature;
		tmp_state.power =  begin_state.power - wait_time * robot_.cold_down_power;
		tmp_state.time = begin_state.time + wait_time;
		robot_state_vec.push_back(tmp_state);

		//end move process
		out_state.process_type = ProcessType::MOVING;
		out_state.temperature = robot_.max_temperature;
		out_state.power = tmp_state.power - traj.totalTime() * robot_.move_power;
		out_state.time = tmp_state.time + traj.totalTime();

	}else{
		//begin move process
		RobotState tmp_state = begin_state;
		tmp_state.process_type = ProcessType::MOVING;
		robot_state_vec.push_back(tmp_state);

		//end move process
		out_state.process_type = ProcessType::MOVING;
		out_state.temperature = need_temperature;
		out_state.power -= traj.totalTime() * robot_.move_power;
		out_state.time += traj.totalTime();
	}	
	return out_state;
}

bool RobotTask::PreconValid(const vector<string>& precons, 
							const vector<int>& sp,
							const vector<int>& ep,
							const RobotState& cur_state){
	for(auto& precon : precons) {
		if(precon == "path_valid") {
			Trajectory traj(sp, ep, voxel_map_, robot_.velocity);
			if(!traj.isValid()){
				std::cout<<"Traj isn't valid "<<std::endl;
				return false;
			} 
		}
		if(precon == "robot_has_soil" && cur_state.material_type != RobotMaterial::SOIL){
			return false;
		}
		if(precon == "robot_has_cup" && cur_state.material_type != RobotMaterial::CUP) {
			return false;
		}
		if(precon == "robot_has_brick" && cur_state.material_type != RobotMaterial::BRICK) {
			return false;
		}
		if(precon == "zlq_out_has_cup"){
			return (!data_pool_->zlq_out_material.cup.empty() && cur_state.material_type == RobotMaterial::EMPTY);
		}
		if(precon == "zlq_out_has_brick"){
			return (!data_pool_->zlq_out_material.brick.empty() && cur_state.material_type == RobotMaterial::EMPTY);
		}
		
		//ZLQ任务前置条件,global task 过滤了
		if(precon == "zlq_in_has_soil"){
			return (data_pool_->zlq_in_material.soil > 0.0);
		}
		if(precon == "zlq_in_has_cup"){
			return (data_pool_->zlq_in_material.cup_soil > 0.0);
		}
	}
	return true;

}

void RobotTask::UpdateState(const TaskConfig& next_task_param, std::shared_ptr<TaskNode> next_node){
	if(next_task_param.name == "get_soil"){
		next_node->end_state.material_type = RobotMaterial::SOIL;
	}
	if(next_task_param.name == "fill_soil"){
		next_node->end_state.material_type = RobotMaterial::EMPTY;
		data_pool_->zlq_out_material.cup.insert(next_node->end_state.time);
	}
	if(next_task_param.name == "get_cup"){
		next_node->end_state.material_type = RobotMaterial::CUP;
		if(!data_pool_->zlq_out_material.cup.empty()) {
			data_pool_->zlq_out_material.cup.erase(data_pool_->zlq_out_material.cup.begin());
		}
	}
	if(next_task_param.name == "fill_cup"){
		next_node->end_state.material_type = RobotMaterial::EMPTY;
		data_pool_->zlq_out_material.brick.insert(next_node->end_state.time);
	}
	if(next_task_param.name == "get_brick"){
		next_node->end_state.material_type = RobotMaterial::BRICK;
		if(!data_pool_->zlq_out_material.brick.empty()) {
			data_pool_->zlq_out_material.brick.erase(data_pool_->zlq_out_material.brick.begin());
		}
	}
	if(next_task_param.name == "build_brick"){
		next_node->end_state.material_type = RobotMaterial::EMPTY;
	}
	// TODO: zlq筛土和制砖过程
	// if(next_task_param.name == "sieving_soil"){
	// 	data_pool_->zlq_in_material.soil -= 1.0;
	// 	if(data_pool_->zlq_in_material.soil < 0.0) data_pool_->zlq_in_material.soil = 0.0;
	// 	data_pool_->zlq_out_material.cup.insert(next_node->end_state.time);
	// }
	// if(next_task_param.name == "generate_brick"){
	// 	data_pool_->zlq_in_material.cup_soil -= 1.0;
	// 	if(data_pool_->zlq_in_material.cup_soil < 0.0) data_pool_->zlq_in_material.cup_soil = 0.0;
	// 	data_pool_->zlq_out_material.brick.insert(next_node->end_state.time);
	// }
}

RobotState RobotTask::CalculateOperationalState(const  TaskConfig& next_task_param, const RobotState& begin_state, 
												vector<RobotState>& robot_state_vec) {
	RobotState out_state = begin_state;
	if(next_task_param.name != "charge"){
		double need_temperature = begin_state.temperature + next_task_param.temperature;
		if(need_temperature > robot_.max_temperature){
			//begin cold down process
			RobotState tmp_state = begin_state;
			tmp_state.process_type = ProcessType::WAIT;
			robot_state_vec.push_back(tmp_state);
			//begin operating process
			double wait_time = (need_temperature - robot_.max_temperature) / fabs(robot_.cold_down_temperature);
			tmp_state.process_type = ProcessType::OPERATING;
			tmp_state.power -= wait_time* robot_.cold_down_power;
			tmp_state.time -= wait_time;
			tmp_state.temperature = robot_.max_temperature - next_task_param.temperature;
			robot_state_vec.push_back(tmp_state);
			//end operating process
			out_state.process_type = ProcessType::OPERATING;
			out_state.temperature = robot_.max_temperature;
			out_state.power -= wait_time * robot_.cold_down_power + next_task_param.power;
			out_state.time += wait_time + next_task_param.time;
		}else{
			//begin operating process
			RobotState tmp_state = begin_state;
			tmp_state.process_type = ProcessType::OPERATING;
			robot_state_vec.push_back(tmp_state);
			//end operating process
			out_state.process_type = ProcessType::OPERATING;
			out_state.temperature = need_temperature;
			out_state.power -= next_task_param.power;
			out_state.time += next_task_param.time;
		}
	}else{
		RobotState tmp_state = begin_state;
		double wait_time = 0.0;
		if(begin_state.temperature > 15.0){
			wait_time = (begin_state.temperature - 15.0) / fabs(robot_.cold_down_temperature);
			if(begin_state.power - wait_time * robot_.cold_down_power < 0.3){
				wait_time = (begin_state.power - 0.3) / fabs(robot_.cold_down_power);
			}
		}
		if(wait_time <= 0.0){
			wait_time = 0.0;
			tmp_state.process_type = ProcessType::OPERATING;
			robot_state_vec.push_back(tmp_state);
		}else{
			tmp_state.process_type = ProcessType::WAIT;
			robot_state_vec.push_back(tmp_state);
			tmp_state.process_type = ProcessType::OPERATING;
			tmp_state.time = begin_state.time + wait_time;
			tmp_state.temperature = begin_state.temperature + wait_time * robot_.cold_down_temperature;
			tmp_state.power = begin_state.power - wait_time * robot_.cold_down_power;
			robot_state_vec.push_back(tmp_state);
		}

		out_state.power -= wait_time * robot_.cold_down_power; 
		out_state.time = out_state.time  - (1.0 - out_state.power) / next_task_param.power + wait_time;
		out_state.temperature = begin_state.temperature + wait_time * robot_.cold_down_temperature;
		out_state.power = 1.0;
		double charge_time = (1.0 - out_state.power) / next_task_param.power;
	}
	return out_state;
}


void RobotTask::GetInfo(){
	//publish path
	double time_scale = 1/36.0;//for test, trans second to hour
	double current_time = (ros::Time::now().toSec() - start_time_) * time_scale;
	//publish pos
	FindPubTask(current_time);
}

void RobotTask::FindPubTask(double t) {
	if(task_vec_.empty()){
		std::cout<<"task_vec_ is empty"<<std::endl;
		find_task_idx_ = 0;
		return;
	} 
	if(t < task_vec_[0].end_state.time){
		find_task_idx_ = 0;
	}
	if(find_task_idx_ >= 0){
		for(int i = find_task_idx_; i < task_vec_.size()-1; i++){
			if(t > task_vec_[i].end_state.time){
				find_task_idx_ = i+1;
				break;
			}
		}
	}
	TaskConfig task_param;
	if(config_.task_params.count(task_vec_[find_task_idx_].task_name)){
		task_param = config_.task_params.find(task_vec_[find_task_idx_].task_name)->second;
	}else{
		return;
	}
	traj_.clear();

	if(0<=find_task_idx_  && find_task_idx_ <= (task_vec_.size() - 2)) {
		if(task_vec_[find_task_idx_].task_name == "get_brick"
		|| task_vec_[find_task_idx_ + 1].task_name == "get_brick") {
			vis_meterial_ = true;
		}else{
			vis_meterial_ = false;
		}
	}
	for(int i=0; i<task_vec_[find_task_idx_].task_process.process.size(); i++){
		if(i == task_vec_[find_task_idx_].task_process.process.size() - 1){
			if(t > task_vec_[find_task_idx_].task_process.process[i].time && t < task_vec_[find_task_idx_].end_state.time){
				if(task_vec_[find_task_idx_].task_process.process[i].process_type == ProcessType::OPERATING){
					//publish text: operating
					double remain_time = task_vec_[find_task_idx_].end_state.time - t;
					double duration = t - task_vec_[find_task_idx_].task_process.process[i].time;
					double power = task_vec_[find_task_idx_].task_process.process[i].power - task_param.power * duration;
					double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * task_param.temperature;
					robot_text_ = "Type: "+task_vec_[find_task_idx_].task_name+"\n"
								+"Operating: "+ std::to_string(remain_time)+"\n"
								+"Power: "+std::to_string(power)
								+"\nTemper: "+ std::to_string(temperature);
				}
			}
			break;
		}
		if(t >= task_vec_[find_task_idx_].task_process.process[i].time && 
			t < task_vec_[find_task_idx_].task_process.process[i+1].time){
			if(task_vec_[find_task_idx_].task_process.process[i].process_type == ProcessType::WAIT){
				//publish text: wait to cold down
				double remain_time = task_vec_[find_task_idx_].task_process.process[i+1].time - t;
				double duration = t - task_vec_[find_task_idx_].task_process.process[i].time;
				double power = task_vec_[find_task_idx_].task_process.process[i].power - robot_.cold_down_power * duration;
				double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * robot_.cold_down_temperature;
				robot_text_ = "Type: "+task_vec_[find_task_idx_].task_name+"\n"
											+"Cold down: "+ std::to_string(remain_time)
											+"\nPower: "+std::to_string(power)
											+"\nTemper: "+ std::to_string(temperature);
			}
			if(task_vec_[find_task_idx_].task_process.process[i].process_type == ProcessType::MOVING){
				//publish text: moving
				double time = t - task_vec_[find_task_idx_].task_process.process[i].time;
				double totalTime = task_vec_[find_task_idx_].task_process.move_traj.totalTime();
				if(time < totalTime){
					pos_ = task_vec_[find_task_idx_].task_process.move_traj.getPos(time);
				}
				double duration = t - task_vec_[find_task_idx_].task_process.process[i].time;
				double power = task_vec_[find_task_idx_].task_process.process[i].power - robot_.move_power * duration;
				double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * robot_.move_temperature;
				robot_text_ = "Type: "+task_vec_[find_task_idx_].task_name+"\n"
							+"Moving: "+to_string(totalTime - time)
							+"\nPower: "+std::to_string(power)
							+"\nTemper: "+ std::to_string(temperature);
				// for(double traj_t = 0.0; traj_t < totalTime; traj_t += 0.001){
				// 	traj_.push_back(task_vec_[find_task_idx_].task_process.move_traj.getPos(traj_t));
				// }
				traj_ = task_vec_[find_task_idx_].task_process.move_traj.Path();
			}
		}
	}
} 



