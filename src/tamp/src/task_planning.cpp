#include <cassert>
#include "task_planning.hpp"

#include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

TaskPlanning::TaskPlanning(){

}
TaskPlanning::TaskPlanning(ros::NodeHandle &nh) {
	nh_ = nh;
	map_initialized_ = false;

	double xl = -6.0;
	double xu = 6.0;
	double yl = -6.0;
	double yu = 6.0;
	double zl = 0.0;
	double zu = 2.0;
	double voxelWidth = 0.25;
  const Eigen::Vector3i xyz((xu - xl) / voxelWidth,(yu - yl) / voxelWidth,(zu - zl) / voxelWidth);
  std::cout<<"xyz "<<xyz<<std::endl;
  const Eigen::Vector3d offset(xl, yl, zl);
  voxel_map_ = VoxelMap(xyz, offset, voxelWidth);
	map_sub_ = nh_.subscribe("mock_map", 1000, &TaskPlanning::mapCallBack, this);
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud2", 1);
	vis_meterial_ = false;
}
TaskPlanning::~TaskPlanning(){

}
void TaskPlanning::Init(Config& config){
	start_time_ = ros::Time::now().toSec();
	pos_ = Eigen::Vector3d(0,-2.0,0);
	robot_param_.move_temperature = config.move_temperature;
	robot_param_.max_velocity = config.max_velocity;
	robot_param_.move_power = config.move_power;
	robot_param_.max_temperature = config.max_temperature;
	robot_param_.cold_down_temperature = config.cold_down_temperature;
	robot_param_.cold_down_power = config.cold_down_power;

	robot_current_task_.name = "none";
	robot_current_task_.task_type = NONE;
	robot_current_task_.param.spend_time = 0;
	robot_current_task_.param.spend_power = 0;
	robot_current_task_.param.temperature = 0;
	robot_current_task_.end_state = RobotState(0.0, 1.0, 15.0);
	after_charged_task_ = &robot_current_task_;

	get_soil_task_.name = "get soil";
	get_soil_task_.task_type = GET_SOIL;
	get_soil_task_.pos = config.get_soil_pos;
	get_soil_task_.param.spend_time = config.get_soil_time;
	get_soil_task_.param.spend_power = config.get_soil_power;
	get_soil_task_.param.temperature = config.get_soil_temperature;

	fill_soil_task_.name = "fill soil";
	fill_soil_task_.task_type = FILL_SOIL;
	fill_soil_task_.pos = config.fill_soil_pos;
	fill_soil_task_.param.spend_time = config.fill_soil_time;
	fill_soil_task_.param.spend_power = config.fill_soil_power;
	fill_soil_task_.param.temperature = config.fill_soil_temperature;

	get_cup_task_.name = "get cup";
	get_cup_task_.task_type = GET_CUP;
	get_cup_task_.pos = config.get_cup_pos;
	get_cup_task_.param.spend_time = config.get_cup_time;
	get_cup_task_.param.spend_power = config.get_cup_power;
	get_cup_task_.param.temperature = config.get_cup_temperature;

	fill_cup_task_.name = "fill cup";
	fill_cup_task_.task_type = FILL_CUP;
	fill_cup_task_.pos = config.fill_cup_pos;
	fill_cup_task_.param.spend_time = config.fill_cup_time;
	fill_cup_task_.param.spend_power = config.fill_cup_power;
	fill_cup_task_.param.temperature = config.fill_cup_temperature;

	get_brick_task_.name = "get brick";
	get_brick_task_.task_type = GET_BRICK;
	get_brick_task_.pos = config.get_brick_pos;
	get_brick_task_.param.spend_time = config.get_brick_time;
	get_brick_task_.param.spend_power = config.get_brick_power;
	get_brick_task_.param.temperature = config.get_brick_temperature;

	build_brick_task_.name = "build brick";
	build_brick_task_.task_type = BUILD_BRICK;
	build_brick_task_.pos = config.build_brick_pos;
	build_brick_task_.param.spend_time = config.build_brick_time;
	build_brick_task_.param.spend_power = config.build_brick_power;
	build_brick_task_.param.temperature = config.build_brick_temperature;

	charge_task_.name = "charge";
	charge_task_.task_type = CHARGE;
	charge_task_.pos = config.charge_pos;
	charge_task_.param.spend_power = config.charge_power_speed;
	charge_task_.param.temperature = 0.0;
}

void TaskPlanning::Run(){
	if(!map_initialized_){
		return;
	}
	if(robot_current_task_.end_state.time > 12.0){
		// cout<<"Has worked 12h, end task"<<endl;
		return;
	}
	switch (robot_current_task_.task_type){
		case NONE: {
			// robot_current_task_.end_state.power = 0.5;
			UpdateTask(get_soil_task_);
			after_charged_task_ = &get_soil_task_;
			break;
		}
		case TaskType::GET_SOIL : {
			UpdateTask(fill_soil_task_);
			after_charged_task_ = &fill_soil_task_;
			break;
		}
		case TaskType::FILL_SOIL : {
			UpdateTask(get_cup_task_);
			after_charged_task_ = &get_cup_task_;
			break;
		}
		case TaskType::GET_CUP : {
			UpdateTask(fill_cup_task_);
			after_charged_task_ = &fill_cup_task_;
			make_brick_task_.beginOperate();
			break;
		}
		case TaskType::FILL_CUP : {
			std::cout<<"has_meterial "<<make_brick_task_.has_meterial<<std::endl;
			if(make_brick_task_.has_meterial){
				UpdateTask(get_brick_task_);
				make_brick_task_.resetCounter();
				after_charged_task_ = &get_brick_task_;
			} else{
				UpdateTask(get_soil_task_);
				after_charged_task_ = &get_soil_task_;
			} 
			break;
		}
		case TaskType::GET_BRICK : {
			UpdateTask(build_brick_task_);
			after_charged_task_ = &build_brick_task_;
			break;
		}
		case TaskType::BUILD_BRICK : {
			UpdateTask(get_soil_task_);
			after_charged_task_ = &build_brick_task_;
			break;
		}
		case TaskType::CHARGE : {
			if(robot_current_task_.end_state.power > 0.999 && after_charged_task_ != nullptr){
				UpdateTask(*after_charged_task_);
			}else{
				robot_info_.power = 1.0;
			}
			break;
		}
	}
	if(make_brick_task_.isOperating())make_brick_task_.countFunction();
	task_vec_.push_back(robot_current_task_);
	cout<<"task name: "<<robot_current_task_.name <<"; "
	<<" end time: "<< robot_current_task_.end_state.time<<"; "
	<<" end power: "<<robot_current_task_.end_state.power<<"; "
	<<" end temperature: "<<robot_current_task_.end_state.temperature<<"; "
	<<" process action: "<< robot_current_task_.task_process.process.size()<< endl;
	// std::cin.get();
}

void TaskPlanning::UpdateTask(const RobotTask& next_task){
	robot_current_task_.task_process.process.clear();
	RobotState begin_task_state = robot_current_task_.end_state;
	vector<RobotState> next_task_process_vec;
	Trajectory traj_cur_nxt(robot_current_task_.pos, next_task.pos, &voxel_map_, robot_param_.max_velocity);
	RobotState after_move_state = CalculateState(traj_cur_nxt, begin_task_state, next_task_process_vec);
	RobotState after_operate_state = CalculateOperationalState(next_task, after_move_state, next_task_process_vec);
	vector<RobotState> tmp_vec;
	Trajectory traj_nxt_chg(next_task.pos, charge_task_.pos, &voxel_map_, robot_param_.max_velocity);
	RobotState after_mvcharge_state = CalculateState(traj_nxt_chg, after_operate_state, tmp_vec);
	if(after_mvcharge_state.power > 0.3){
		robot_current_task_.task_process.process = next_task_process_vec;
		robot_current_task_.getInput(next_task);
		robot_current_task_.end_state = after_operate_state;
		robot_current_task_.charge_traj = traj_nxt_chg;
		robot_current_task_.task_process.move_traj = traj_cur_nxt;
	}else{
		RobotState begin_state = robot_current_task_.end_state;
		Trajectory charge_traj(robot_current_task_.pos, charge_task_.pos, &voxel_map_, robot_param_.max_velocity);
		// Trajectory charge_traj = robot_current_task_.charge_traj;
		RobotState after_move_state_ch = CalculateState(charge_traj, begin_state, robot_current_task_.task_process.process);
		RobotState after_charge_state_ch = CalculateOperationalState(charge_task_, after_move_state_ch, robot_current_task_.task_process.process);
		robot_current_task_.getInput(charge_task_);
		robot_current_task_.task_process.move_traj = charge_traj;
		robot_current_task_.end_state = after_charge_state_ch;
	}
}

RobotState TaskPlanning::CalculateState(const Trajectory& traj, const RobotState& begin_state, 
																				vector<RobotState>& robot_state_vec) {
	RobotState out_state = begin_state;
	double need_temperature = begin_state.temperature + traj.totalTime() * robot_param_.move_temperature;
	if(need_temperature > robot_param_.max_temperature){
		//begin cold down process
		RobotState tmp_state = begin_state;
		tmp_state.process_type = ProcessType::WAIT;
		robot_state_vec.push_back(tmp_state);
		
		//begin move process
		double wait_time = (need_temperature - robot_param_.max_temperature) / fabs(robot_param_.cold_down_temperature);
		tmp_state.process_type = ProcessType::MOVING;
		tmp_state.temperature = robot_param_.max_temperature - traj.totalTime() * robot_param_.move_temperature;
		tmp_state.power =  begin_state.power - wait_time * robot_param_.cold_down_power;
		tmp_state.time = begin_state.time + wait_time;
		robot_state_vec.push_back(tmp_state);

		//end move process
		out_state.process_type = ProcessType::MOVING;
		out_state.temperature = robot_param_.max_temperature;
		out_state.power = tmp_state.power - traj.totalTime() * robot_param_.move_power;
		out_state.time = tmp_state.time + traj.totalTime();

	}else{
		//begin move process
		RobotState tmp_state = begin_state;
		tmp_state.process_type = ProcessType::MOVING;
		robot_state_vec.push_back(tmp_state);

		//end move process
		out_state.process_type = ProcessType::MOVING;
		out_state.temperature = need_temperature;
		out_state.power -= traj.totalTime() * robot_param_.move_power;
		out_state.time += traj.totalTime();
	}	
	return out_state;
}

RobotState TaskPlanning::CalculateOperationalState(const RobotTask& next_task, const RobotState& begin_state, 
																										vector<RobotState>& robot_state_vec) {
	RobotState out_state = begin_state;
	if(next_task.task_type != TaskType::CHARGE){
		double need_temperature = begin_state.temperature + next_task.param.temperature;
		if(need_temperature > robot_param_.max_temperature){
			//begin cold down process
			RobotState tmp_state = begin_state;
			tmp_state.process_type = ProcessType::WAIT;
			robot_state_vec.push_back(tmp_state);
			//begin operating process
			double wait_time = (need_temperature - robot_param_.max_temperature) / fabs(robot_param_.cold_down_temperature);
			tmp_state.process_type = ProcessType::OPERATING;
			tmp_state.power -= wait_time* robot_param_.cold_down_power;
			tmp_state.time -= wait_time;
			tmp_state.temperature = robot_param_.max_temperature - next_task.param.temperature;
			robot_state_vec.push_back(tmp_state);
			//end operating process
			out_state.process_type = ProcessType::OPERATING;
			out_state.temperature = robot_param_.max_temperature;
			out_state.power -= wait_time * robot_param_.cold_down_power + next_task.param.spend_power;
			out_state.time += wait_time + next_task.param.spend_time;
		}else{
			//begin operating process
			RobotState tmp_state = begin_state;
			tmp_state.process_type = ProcessType::OPERATING;
			robot_state_vec.push_back(tmp_state);
			//end operating process
			out_state.process_type = ProcessType::OPERATING;
			out_state.temperature = need_temperature;
			out_state.power -= next_task.param.spend_power;
			out_state.time += next_task.param.spend_time;
		}
	}else{
		RobotState tmp_state = begin_state;
		double wait_time = 0.0;
		if(begin_state.temperature > 15.0){
			wait_time = (begin_state.temperature - 15.0) / fabs(robot_param_.cold_down_temperature);
			if(begin_state.power - wait_time * robot_param_.cold_down_power < 0.3){
				wait_time = (begin_state.power - 0.3) / fabs(robot_param_.cold_down_power);
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
			tmp_state.temperature = begin_state.temperature + wait_time * robot_param_.cold_down_temperature;
			tmp_state.power = begin_state.power - wait_time * robot_param_.cold_down_power;
			robot_state_vec.push_back(tmp_state);
		}

		out_state.power -= wait_time * robot_param_.cold_down_power; 
		out_state.time = out_state.time  - (1.0 - out_state.power) / next_task.param.spend_power + wait_time;
		out_state.temperature = begin_state.temperature + wait_time * robot_param_.cold_down_temperature;
		out_state.power = 1.0;
		double charge_time = (1.0 - out_state.power) / next_task.param.spend_power;
	}
	return out_state;
}

void TaskPlanning::GetInfo(){
	//publish path
	double time_scale = 1/36.0;//for test, trans second to hour
	double current_time = (ros::Time::now().toSec() - start_time_) * time_scale;
	//publish pos
	FindPubTask(current_time);
}

void TaskPlanning::FindPubTask(double t) {
	if(task_vec_.empty()){
		find_task_idx_ = -1;
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
	traj_.clear();


	if(0<=find_task_idx_  && find_task_idx_ <= (task_vec_.size() - 2)) {
		if(task_vec_[find_task_idx_].task_type == TaskType::GET_BRICK
		|| task_vec_[find_task_idx_ + 1].task_type == TaskType::GET_BRICK) {
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
					double power = task_vec_[find_task_idx_].task_process.process[i].power - task_vec_[find_task_idx_].param.spend_power * duration;
					double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * task_vec_[find_task_idx_].param.temperature;
					robot_text_ = "Type: "+task_vec_[find_task_idx_].name+"\n"
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
				double power = task_vec_[find_task_idx_].task_process.process[i].power - robot_param_.cold_down_power * duration;
				double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * robot_param_.cold_down_temperature;
				robot_text_ = "Type: "+task_vec_[find_task_idx_].name+"\n"
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
				double power = task_vec_[find_task_idx_].task_process.process[i].power - robot_param_.move_power * duration;
				double temperature = task_vec_[find_task_idx_].task_process.process[i].temperature + duration * robot_param_.move_temperature;
				robot_text_ = "Type: "+task_vec_[find_task_idx_].name+"\n"
							+"Moving: "+to_string(totalTime - time)
							+"\nPower: "+std::to_string(power)
							+"\nTemper: "+ std::to_string(temperature);
				// for(double traj_t = 0.0; traj_t < totalTime; traj_t += 0.001){
				// 	traj_.push_back(task_vec_[find_task_idx_].task_process.move_traj.getPos(traj_t));
				// }
				traj_ = task_vec_[find_task_idx_].task_process.move_traj.Path();
				// std::cout<<robot_text_ <<" traj_ "<<traj_.size()<<std::endl;
			}
		}
	}
} 

void TaskPlanning::mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!map_initialized_)
  {
  	// visiualize
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // cloud.height = 1;  // 设置点云高度（1表示无序点云）
    // sensor_msgs::PointCloud2 cloud_msg;
    size_t cur = 0;
    const size_t total = msg->data.size() / msg->point_step;
    float *fdata = (float *)(&msg->data[0]);
    for (size_t i = 0; i < total; i++)
    {
      cur = msg->point_step / sizeof(float) * i;
      if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
          std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
          std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
      {
          continue;
      }
      // std::cout<<"fdata "<<fdata[cur + 0]<<" "<<fdata[cur + 1]<<" "<<fdata[cur + 2]<<std::endl;
      voxel_map_.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                           fdata[cur + 1],
                                           fdata[cur + 2]));
      // cloud.points.push_back(pcl::PointXYZ(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
    }
    // // if(map_ptr->query(position) == 0)
    // cloud.width = cloud.points.size();
    // pcl::toROSMsg(cloud, cloud_msg);
    // cloud_msg.header.frame_id = "odom";
    // cloud_msg.header.stamp = ros::Time::now(); // 设置时间戳
    // pub_.publish(cloud_msg); 

    voxel_map_.dilate(std::ceil(0.5 / voxel_map_.getScale()));
    map_initialized_ = true;
  }
  // for(double x = -21.0; x< 21.0; x+=2){
  // 	for(double y = -21.0; y< 21.0; y+=2){
  // 		// for(double z = 0.0;)
  // 		bool occy = voxel_map_.query(Eigen::Vector3d(x,y,0.0));
  // 		if(occy) std::cout<<"pos "<<x<<" "<<y<<" occy "<<occy<<std::endl;

  // 	}
  // }
}