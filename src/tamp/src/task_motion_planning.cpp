#include "task_planning.hpp"
#include "visualizer.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
using namespace std;
void ACallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  std::cout<<"mapCallBack 11"<<std::endl;
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "tamp");
  ros::NodeHandle nh_;
  Visualizer visualizer(nh_);
  Config config_(ros::NodeHandle("~"));
  config_.get_soil_pos = visualizer.SoilRegion();
  config_.fill_soil_pos = visualizer.SendSoilPos();
  config_.get_cup_pos = visualizer.GetCupPos();
  config_.fill_cup_pos = visualizer.SendCupPos();
  config_.get_brick_pos = visualizer.GetBrickPos();
  config_.charge_pos = visualizer.ChargePos();
  config_.build_brick_pos = visualizer.BuildRegion();
  TaskPlanning task_planner(nh_);
  task_planner.Init(config_);

  ros::Subscriber map_sub_ = nh_.subscribe("voxel_map", 1000, &ACallBack);
  std::cout<<"subscribe "<<std::endl;
  ros::Rate rate(20);  // 1 Hz
  while (ros::ok())
  {
    task_planner.Run();
    task_planner.GetInfo();
    visualizer.setPos(task_planner.getPos());
    visualizer.setText(task_planner.getText());
    visualizer.setTraj(task_planner.getTraj());
    visualizer.setTask(task_planner.getMeterial());
    visualizer.Publish();
    rate.sleep();
    ros::spinOnce();
  }
	return 0;
}

