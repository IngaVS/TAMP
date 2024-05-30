#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "hybridAStar.hpp"
using namespace std;
class Trajectory {
public:
	Trajectory();
	Trajectory(const Eigen::Vector3d& sp, const Eigen::Vector3d& ep, const VoxelMap* map_ptr, const double max_vel);
	~Trajectory();
public:
	Eigen::Vector3d getPos(const double t) const;
	double totalTime() const;
	vector<Eigen::Vector3d> Path() const;

private:
	Eigen::Vector3d start_;
	Eigen::Vector3d goal_;
	double speed_;
	vector<Eigen::Vector3d> path_;
	map<double, Node> with_len_path_;

	void Interpolation(const std::vector<Node>& path);
};