#include "motion_planning.hpp"

Trajectory::Trajectory(){

}
Trajectory::Trajectory(const Eigen::Vector3d& sp, const Eigen::Vector3d& ep, const VoxelMap* map_ptr, const double max_vel) {
	start_ = sp;
	goal_ = ep;
	speed_ = max_vel;
	path_.clear();
	Eigen::Vector3d ori = Eigen::Vector3d(ep[0], ep[1], 0.0) - Eigen::Vector3d(sp[0], sp[1], 0.0);
    Eigen::Vector3d axis_x(1,0,0);
    double dotp =  ori.dot(axis_x) / ori.norm();
    auto crop =  (ori[0] * axis_x[1] - ori[1] * axis_x[0]);
    double yaw = crop > 0.0 ? -std::acos(dotp) : std::acos(dotp);
	Interpolation(astar(map_ptr, Point(sp[0],sp[1], yaw), Point(ep[0],ep[1], yaw)));
}

void Trajectory::Interpolation(const std::vector<Node>& path){
	if(path.size() < 2) return;
	double wheel_base = 1.0;
	double length = 0.0;
	with_len_path_.clear();
	for(auto itr = next(path.begin()); itr != path.end(); itr++) {
		Eigen::Vector3d st = Eigen::Vector3d(std::prev(itr)->point.x, std::prev(itr)->point.y, std::prev(itr)->point.yaw);
		Eigen::Vector3d ed = Eigen::Vector3d(itr->point.x, itr->point.y, itr->point.yaw);
		
		with_len_path_.insert(make_pair(length, *std::prev(itr)));
		path_.push_back(Eigen::Vector3d(std::prev(itr)->point.x, std::prev(itr)->point.y, 0.0));
		Eigen::Vector3d insert_pt = st;
		Node insert_node = *std::prev(itr);
		for(int i=1; i < 5; i++){
			Eigen::Vector3d diff = (ed - st) * 0.2;
			insert_pt += diff;
			insert_node.point.x = insert_pt[0];
			insert_node.point.y = insert_pt[1];
			insert_node.point.yaw = insert_pt[2];
			length += std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
			with_len_path_.insert(make_pair(length, insert_node));
			path_.push_back(Eigen::Vector3d(insert_pt[0], insert_pt[1], 0.0));
		}
	}
	// for(auto& pt : path) {
	//     int nsteps = 10;
	//     Node start_nd =  pt;
	//     with_len_path_.insert(make_pair(length, start_nd));
	//     path_.push_back(Eigen::Vector3d(start_nd.point.x, start_nd.point.y, 0.0));
	// 	for(int i=1; i<nsteps; i++)
	// 	{
	// 	    start_nd.point.yaw +=  start_nd.input.vel * tan(start_nd.input.delta) / wheel_base * 0.1;
	// 	    start_nd.point.x +=  start_nd.input.vel * cos(start_nd.point.yaw) * 0.1;
	// 	    start_nd.point.y +=  start_nd.input.vel * sin(start_nd.point.yaw) * 0.1;
	// 	    length += std::sqrt(std::pow(start_nd.input.vel * cos(start_nd.point.yaw) * 0.1, 2) +
	// 	    			   		std::pow(start_nd.input.vel * sin(start_nd.point.yaw) * 0.1, 2));
	// 	    with_len_path_.insert(make_pair(length, start_nd));
	// 	    path_.push_back(Eigen::Vector3d(start_nd.point.x, start_nd.point.y, 0.0));
	// 	    // path.push_back(start_nd);
	// 	}
	// }
}

Trajectory::~Trajectory(){

}

Eigen::Vector3d Trajectory::getPos(const double t) const{
	if(t < totalTime() && t > 0.0 && !with_len_path_.empty()){
		double s = speed_ * t;
		auto itr = with_len_path_.upper_bound(s);
		if(itr != with_len_path_.end() && itr != with_len_path_.begin()) {
			Eigen::Vector3d st = Eigen::Vector3d(std::prev(itr)->second.point.x,
				std::prev(itr)->second.point.y,std::prev(itr)->second.point.yaw);
			Eigen::Vector3d ed = Eigen::Vector3d(itr->second.point.x,
							itr->second.point.y,itr->second.point.yaw);
			Eigen::Vector3d pos = st +  (ed - st) * (s - std::prev(itr)->first) / (itr->first - std::prev(itr)->first);
			return pos;
		}
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}else{
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}
}

double Trajectory::totalTime() const{
	// return (goal_ - start_).norm() / speed_;
	if(with_len_path_.empty()) return 0.0;
	else {
		return with_len_path_.rbegin()->first / speed_;
	}
}

vector<Eigen::Vector3d> Trajectory::Path() const {
	return path_;
}