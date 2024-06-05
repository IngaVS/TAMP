#include "global_task.hpp"
GlobalTask::GlobalTask(){

}
~GlobalTask::GlobalTask(){

}

void GlobalTask::GlobalTaskOptimal() {
	//根据着陆器状态和机器人个数，计算机器人任务分配
	for(auto robot_types : config_.robots){
		if(robot_types.task_file == "ZLQTask") continue;
		TaskGraph* task_graph = &config_.task_graphs.find(robot_types.task_file)->second;
		for(int i=0; i< robot_types.robot_num; i++){
			RobotTask robot_task;
			robot_task.Init(*task_graph, robot_types);
			robot_task.Run();
		}
	}
}