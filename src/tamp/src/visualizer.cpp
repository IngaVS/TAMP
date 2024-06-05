#include "visualizer.hpp"

Visualizer::Visualizer(){

}
Visualizer::~Visualizer(){

}
Visualizer::Visualizer(ros::NodeHandle &nh_)
    : nh(nh_)
{
  stl_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("stl_model", 10);
  zlq_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("zlq_model", 10);
  soil_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("soil", 10);
  build_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("build", 10);
  // robot_pos_tmp = Eigen::Vector3d(0.0, -2.0, 0.0);
  meterial_ = false;
  last_meterial_ = false;
  robot_heading_ = 0.0;
  ZLQPub();
  SoilPub();
  BuildPub();
}
void Visualizer::Publish(){
  VehiclePub();
  if(meterial_ != last_meterial_){
    ZLQPub();
  }
  last_meterial_ = meterial_;
  stl_pub_.publish(slt_marker_);
  zlq_pub_.publish(zlq_marker_);
  soil_pub_.publish(soil_marker_);
  build_pub_.publish(build_marker_);
}

// Convert heading to quaternion
geometry_msgs::Quaternion Visualizer::headingToQuaternion(double heading) {
  tf2::Quaternion quat;
  quat.setRPY(0, 0, heading);
  geometry_msgs::Quaternion orientation;
  tf2::convert(quat, orientation);
  return orientation;
}
void Visualizer::VehiclePub() {
  slt_marker_.markers.clear();
  for(int i = 0; i < robot_num_; i++){
    Eigen::Vector3d robot_pos_tmp = robot_pos_[i];
    std::vector<Eigen::Vector3d> traj_tmp = traj_[i];


    ros::Time timestamp = ros::Time::now();
    visualization_msgs::Marker tmp_marker;
    tmp_marker.header.frame_id = "odom"; // 设置模型的参考坐标系
    tmp_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    tmp_marker.mesh_resource = "package://tamp/stl/ackermann_model.STL"; // 设置STL模型的文件路径
    tmp_marker.ns = "slt_model";
    tmp_marker.id = i*robot_num_ + 1;
    tmp_marker.pose.position.x = robot_pos_tmp[0]; // 设置模型的初始位置
    tmp_marker.pose.position.y = robot_pos_tmp[1];
    tmp_marker.pose.position.z = 0.0;
    
    Eigen::Quaterniond quaternion;
    robot_pos_tmp[2] += M_PI / 2.0;
    // std::cout<<"yaw "<<robot_pos_tmp[2] * 180 / M_PI<<std::endl;
    quaternion = Eigen::AngleAxisd(robot_pos_tmp[2], Eigen::Vector3d::UnitZ());

    tmp_marker.pose.orientation.x = quaternion.x(); // 设置模型的初始姿态
    tmp_marker.pose.orientation.y = quaternion.y();
    tmp_marker.pose.orientation.z = quaternion.z();
    tmp_marker.pose.orientation.w = quaternion.w();
    tmp_marker.scale.x = 0.0002; // 设置模型的尺寸
    tmp_marker.scale.y = 0.0002;
    tmp_marker.scale.z = 0.0002;
    tmp_marker.color.a = 1.0; // 设置模型的透明度
    tmp_marker.color.r = 1.0; // 设置模型的颜色
    tmp_marker.color.g = 0.0;
    tmp_marker.color.b = 0.0;
    slt_marker_.markers.push_back(tmp_marker);

    // tmp_marker.header.frame_id = "odom"; // 设置模型的参考坐标系
    tmp_marker.pose.orientation.x = 0.0; // 设置模型的初始姿态
    tmp_marker.pose.orientation.y = 0.0;
    tmp_marker.pose.orientation.z = 0.0;
    tmp_marker.pose.orientation.w = 1.0;
    tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tmp_marker.ns = "slt_text";
    tmp_marker.id = i*robot_num_ + 2;
    tmp_marker.scale.x = 0.5; // 设置模型的尺寸
    tmp_marker.scale.y = 0.5;
    tmp_marker.scale.z = 0.5;
    tmp_marker.color.a = 1.0; // 设置模型的透明度
    tmp_marker.color.r = 0.0; // 设置模型的颜色
    tmp_marker.color.g = 1.0;
    tmp_marker.color.b = 0.0;
    tmp_marker.text = robot_text_[i];
    slt_marker_.markers.push_back(tmp_marker);



    // tmp_marker.pose.orientation.x = 0.0; // 设置模型的初始姿态
    // tmp_marker.pose.orientation.y = 0.0;
    // tmp_marker.pose.orientation.z = 0.0;
    // tmp_marker.pose.orientation.w = 1.0;
    // tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // tmp_marker.ns = "info_text";
    // tmp_marker.id = i*robot_num_ + 4;
    // tmp_marker.scale.x = 1; // 设置模型的尺寸
    // tmp_marker.scale.y = 1;
    // tmp_marker.scale.z = 1;
    // tmp_marker.color.a = 1.0; // 设置模型的透明度
    // tmp_marker.color.r = 0.0; // 设置模型的颜色
    // tmp_marker.color.g = 1.0;
    // tmp_marker.color.b = 0.0;
    // tmp_marker.pose.position.x = soil_pos_x; // 设置模型的初始位置
    // tmp_marker.pose.position.y = soil_pos_y + 10.0;
    // tmp_marker.pose.position.z = 0.0;
    // tmp_marker.text = robot_text_[i];
    // slt_marker_.markers.push_back(tmp_marker);

    if(!traj_tmp.empty()){
      tmp_marker.ns = "traj";
      tmp_marker.type = visualization_msgs::Marker::POINTS;
      tmp_marker.action = visualization_msgs::Marker::ADD;
      tmp_marker.color.a = 1.0; // 设置模型的透明度
      tmp_marker.color.r = 1.0; // 设置模型的颜色
      tmp_marker.color.g = 1.0;
      tmp_marker.color.b = 0.0;
      tmp_marker.scale.x = 0.1; // 设置模型的尺寸
      tmp_marker.scale.y = 0.1;
      tmp_marker.scale.z = 0.1;
      tmp_marker.pose.position.x = 0.0; // 设置模型的初始位置
      tmp_marker.pose.position.y = 0.0;
      tmp_marker.pose.position.z = 0.0;
      tmp_marker.id = i*robot_num_ + 3;
      for(int i = 0; i < traj_tmp.size(); i++){
        geometry_msgs::Point p;
        p.x = traj_tmp[i][0]; // 设置模型的初始位置
        p.y = traj_tmp[i][1];
        p.z = traj_tmp[i][2];
        tmp_marker.points.push_back(p);
      }
    }
    slt_marker_.markers.push_back(tmp_marker);
  }
}

void Visualizer::ZLQPub() {
  zlq_marker_.markers.clear();
  ros::Time timestamp = ros::Time::now();
  visualization_msgs::Marker tmp_marker;
  tmp_marker.header.frame_id = "odom"; 
  tmp_marker.header.stamp = timestamp;
  tmp_marker.ns = "markers";
  tmp_marker.id = 10;
  tmp_marker.type = visualization_msgs::Marker::CYLINDER;
  tmp_marker.action = visualization_msgs::Marker::ADD;
  tmp_marker.scale.x = length;
  tmp_marker.scale.y = length;
  tmp_marker.scale.z = 2.0;
  tmp_marker.color.r = 0.0;
  tmp_marker.color.g = 1.0;
  tmp_marker.color.b = 0.0;
  tmp_marker.color.a = 0.1;
  tmp_marker.pose.position.x = pos_x;
  tmp_marker.pose.position.y = pos_y;
  tmp_marker.pose.position.z = pos_z;
  tmp_marker.pose.orientation.x = 0.0;
  tmp_marker.pose.orientation.y = 0.0;
  tmp_marker.pose.orientation.z = 0.0;
  tmp_marker.pose.orientation.w = 1.0;
  zlq_marker_.markers.push_back(tmp_marker);

  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
  tmp_marker.text = "ZLQ";
  tmp_marker.color.a = 1.0;
  tmp_marker.id = 11;
  zlq_marker_.markers.push_back(tmp_marker);

  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
  tmp_marker.text = "Soil";
  tmp_marker.pose.position.x = soil_pos_x;
  tmp_marker.pose.position.y = soil_pos_y;
  tmp_marker.pose.position.z = 0.0;
  tmp_marker.id = 12;
  zlq_marker_.markers.push_back(tmp_marker);


  // tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
  // tmp_marker.text = robot_text_;
  // tmp_marker.pose.position.x = soil_pos_x;
  // tmp_marker.pose.position.y = soil_pos_y + 10.0;
  // tmp_marker.pose.position.z = 0.0;
  // tmp_marker.id = 14;
  // zlq_marker_.markers.push_back(tmp_marker);

  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
  tmp_marker.text = "Build";
  tmp_marker.pose.position.x = build_pos_x;
  tmp_marker.pose.position.y = build_pos_y;
  tmp_marker.pose.position.z = 0.0;
  tmp_marker.id = 13;
  zlq_marker_.markers.push_back(tmp_marker);

  tmp_marker.type = visualization_msgs::Marker::CUBE;
  tmp_marker.scale.x = 1.0;
  tmp_marker.scale.y = 1.0;
  tmp_marker.scale.z = 1.0;
  tmp_marker.color.r = 1.0; 
  tmp_marker.color.g = 0.0;
  tmp_marker.color.b = 0.0;
  tmp_marker.color.a = 1.0;
  for(int i=0; i<5; i++) {
    tmp_marker.id = i;
    tmp_marker.pose.position.x = pos_x + length / 2.0 * cos(direction * i);
    tmp_marker.pose.position.y = pos_y + length / 2.0 * sin(direction * i);
    if(i ==4) {
      tmp_marker.color.g = 1.0;
      tmp_marker.color.r = 1.0;
    }
    if(i == 3 && meterial_){
      tmp_marker.color.g = 1.0;
      tmp_marker.color.r = 0.0; 
    }
    ZLQ_module_.push_back(Eigen::Vector3d(tmp_marker.pose.position.x, 
      tmp_marker.pose.position.y, tmp_marker.pose.position.z));

    zlq_marker_.markers.push_back(tmp_marker);
  } 
}

void Visualizer::SoilPub() {
  ros::Time timestamp = ros::Time::now();
  soil_marker_.header.frame_id = "odom";
  soil_marker_.header.stamp = timestamp;
  double radius = 3.0; // Radius of the hexagon
  soil_marker_.polygon.points.clear();
  for (int i = 0; i < 6; ++i) {
    geometry_msgs::Point32 vertex;
    vertex.x = soil_pos_x + radius * cos(i * M_PI / 3);
    vertex.y = soil_pos_y + radius * sin(i * M_PI / 3);
    vertex.z = 0.0;
    soil_marker_.polygon.points.push_back(vertex);
  }
}

void Visualizer::BuildPub() {
  ros::Time timestamp = ros::Time::now();
  build_marker_.header.frame_id = "odom";
  build_marker_.header.stamp = timestamp;
  double radius = 2.0; // Radius of the hexagon
  build_marker_.polygon.points.clear();
  for (int i = 0; i < 6; ++i) {
    geometry_msgs::Point32 vertex;
    vertex.x = build_pos_x + radius * cos(i * M_PI / 3);
    vertex.y = build_pos_y + radius * sin(i * M_PI / 3);
    vertex.z = 0.0;
    build_marker_.polygon.points.push_back(vertex);
  }
}
Eigen::Vector3d Visualizer::SendSoilPos() const{
  if(!ZLQ_module_.empty())
    return ZLQ_module_[0];
  else return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Vector3d Visualizer::GetCupPos() const{
  if(!ZLQ_module_.empty())
    return ZLQ_module_[1];
  else return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Vector3d Visualizer::SendCupPos() const{
  if(!ZLQ_module_.empty())
    return ZLQ_module_[2];
  else return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Vector3d Visualizer::GetBrickPos() const{
  if(!ZLQ_module_.empty())
    return ZLQ_module_[3];
  else return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Vector3d Visualizer::ChargePos() const{
  if(!ZLQ_module_.empty())
    return ZLQ_module_[4];
  else return Eigen::Vector3d(0.0, 0.0, 0.0);
}