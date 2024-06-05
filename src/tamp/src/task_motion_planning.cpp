// #include "task_planning.hpp"
#include "visualizer.hpp"
#include "robot_task.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <voxel_map.hpp>
#include <std_msgs/Int8.h>
using namespace std;
void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg, VoxelMap& voxel_map_, bool& map_initialized_)
{
  if (!map_initialized_)
  {
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
      voxel_map_.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                           fdata[cur + 1],
                                           fdata[cur + 2]));
    }
    voxel_map_.dilate(std::ceil(0.5 / voxel_map_.getScale()));
    map_initialized_ = true;
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tamp");
  ros::NodeHandle nh_;
  Visualizer visualizer(nh_);

  //MAP
  bool map_initialized_ = false;
  double xl = -6.0;
  double xu = 6.0;
  double yl = -6.0;
  double yu = 6.0;
  double zl = 0.0;
  double zu = 2.0;
  double voxelWidth = 0.25;
  const Eigen::Vector3i xyz((xu - xl) / voxelWidth,(yu - yl) / voxelWidth,(zu - zl) / voxelWidth);
  const Eigen::Vector3d offset(xl, yl, zl);
  VoxelMap voxel_map_ = VoxelMap(xyz, offset, voxelWidth);

  Config config_(nh_); 
  double start_time = ros::Time::now().toSec();
  vector<RobotTask> robot_tasks_;
  RobotTask robot_task(nh_);
  for(auto& r : config_.robots){
    for(int i =0; i<r.robot_num; i++){
      robot_task.Init(config_.task_graphs.find(r.task_file)->second, r);
      robot_tasks_.push_back(robot_task);
    }
  }

  ros::Subscriber map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/mock_map", 1000, 
                              boost::bind(&mapCallBack, _1, std::ref(voxel_map_),std::ref(map_initialized_)));
  ros::Rate rate(1000);  // 1 Hz
  while (ros::ok())
  {
    if(map_initialized_){
      std::vector<Eigen::Vector3d> robot_pos_;
      std::vector<std::string> robot_text_;
      std::vector<std::vector<Eigen::Vector3d>> traj_;
      for(auto& rt : robot_tasks_){
        rt.SetStartTime(start_time);
        rt.SetMap(&voxel_map_);
        rt.Run();
        rt.GetInfo();
        robot_pos_.push_back(rt.getPos());
        robot_text_.push_back(rt.getText());
        traj_.push_back(rt.getTraj());
      }
      visualizer.setRobotNum(robot_tasks_.size());
      visualizer.setPos(robot_pos_);
      visualizer.setText(robot_text_);
      visualizer.setTraj(traj_);
      visualizer.Publish();
    }
    
    ros::spinOnce();
    rate.sleep();
  }
	return 0;
}

