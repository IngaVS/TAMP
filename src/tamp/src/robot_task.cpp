#include "robot_task.hpp"
RobotTask::RobotTask(){

}
RobotTask::RobotTask(ros::NodeHandle &nh){
	nh_ = nh;
}
RobotTask::~RobotTask(){
	delete cur_node_;
}
void RobotTask::Init(const Config& config){
	config_ = config;
	if(config_.task_names.empty()){
	    std::cerr << "Task_names empty" << std::endl;
	    std::abort(); // Abnormally terminate the program
	}
	auto ptr = config_.task_params.find(config_.task_names[0]);
	if(ptr == config_.task_params.end()){
	    std::cerr << "Task_params aren't match name" << std::endl;
	    std::abort(); // Abnormally terminate the program
	}
	cur_node_ = new TaskNode(ptr);

	// 打印全局参数
    ROS_INFO("Velocity: %f", config_.velocity);
    ROS_INFO("Move power: %f", config_.move_power);
    ROS_INFO("Move temperature: %f", config_.move_temperature);
    ROS_INFO("Cold down temperature: %f", config_.cold_down_temperature);
    ROS_INFO("Cold down power: %f", config_.cold_down_power);
    ROS_INFO("Max temperature: %f", config_.max_temperature);

    // 打印任务列表
    for (size_t i = 0; i < config_.task_names.size(); ++i) {
        std::string key = config_.task_names[i];
        ROS_INFO("Task %zu:", i+1);
        ROS_INFO("  Name: %s", config_.task_params[key].name.c_str());
        ROS_INFO("  Time: %f", config_.task_params[key].time);
        ROS_INFO("  Power: %f", config_.task_params[key].power);
        ROS_INFO("  Temperature: %f", config_.task_params[key].temperature);
        ROS_INFO("  Position: [%d, %d]", config_.task_params[key].pos[0], config_.task_params[key].pos[1]);
        ROS_INFO("  Neighbors:");
        for (size_t j = 0; j < config_.task_params[key].neighbors.size(); ++j) {
            ROS_INFO("    %s", config_.task_params[key].neighbors[j].c_str());
        }
    }
}
void RobotTask::Run(){
	static int cycle = 0;
	if(cycle > 10) {
		while(cur_node_){
			std::cout<<"task:" << cur_node_->task_ptr->first<<std::endl;
			task_seq_.push_back(*cur_node_);
			cur_node_ = cur_node_->parent;
		}
		return;
	}
	if(cur_node_ == nullptr){
		std::cerr << "cur_node_ is nullptr" << std::endl;
		return;
	}
	if(cur_node_->parent != nullptr){
		cur_node_->parent->expanded_name.insert(cur_node_->task_ptr->first);
	}
	for(string neighbor_name : cur_node_->task_ptr->second.neighbors) {
		if(cur_node_->expanded_name.count(neighbor_name) ) continue;
		auto neighbor_param_ptr = config_.task_params.find(neighbor_name);
		if(neighbor_param_ptr != config_.task_params.end()) {
			TaskNode* new_node = new TaskNode(neighbor_param_ptr);
			auto neighbor_param = neighbor_param_ptr->second;
			new_node->parent = cur_node_;
			new_node->g =  g_func(cur_node_->task_ptr->second, neighbor_param);
			new_node->h = h_func(neighbor_param);
			cur_node_->neighbors.insert(make_pair(new_node->f(), new_node));
		}
	}
	if(cur_node_->neighbors.empty()){
		cur_node_ = cur_node_->parent;
	}else{
		cur_node_ = cur_node_->neighbors.begin()->second;
	}
	cycle++;
}

double RobotTask::g_func(const TaskConfig& st, const TaskConfig& et){
	return 1.0;
}

double RobotTask::h_func(const TaskConfig& et){
	return 1.0;
}