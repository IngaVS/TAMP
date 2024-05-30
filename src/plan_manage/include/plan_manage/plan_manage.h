#ifndef PLAN_MANAGE_HPP
#define PLAN_MANAGE_HPP
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <path_searching/kino_astar.h>
#include <path_searching/astar.h>
#include <tools/visualization.hpp>
#include <tools/CorridorBuilder2d.hpp>
#include <FIRI/interface_FIRI.hpp>
#include <nav_msgs/Odometry.h>
#include <mpc_controller/SE2Traj.h>
#include <mpc_controller/PolyTraj.h>
#include <mpc_controller/SinglePoly.h>
#include <mpc_controller/SinglePolyAC.h>
#include <mpc_controller/PolyTrajAC.h>
#include <mpc_controller/DPtrajContainer.h>

#define BUDGET 0.1
namespace plan_manage{
    class PlanManager{
        public:
            PlanManager(){};
            void init(ros::NodeHandle & nh);
        private:
            bool hasTarget = false, hasOdom = false;
            int plan_id = 0;
            Eigen::Vector3d targetPose, odom;
            std::vector<Eigen::Vector3d> targetPose_vec;
            std::shared_ptr<visualization::Visualization> vis_tool;
            std::shared_ptr<Config> config_;
            std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
            bool first_time = false;
            Eigen::Vector3d first_odom;
            int goal_num = 0;
            std::vector<Eigen::Matrix2Xd> robots_vis_all;
            int goal_list_id = 0;

            map_util::OccMapUtil gridMap;
            Interface_FIRI_Robot interface_FIRI;
            Eigen::Matrix<double, 2, 3> iniState2d, finState2d;
            Eigen::MatrixXd initInnerPts2d;
            std::vector<Eigen::MatrixXd> hPolys;
            std::vector<Eigen::Matrix2Xd> robots;
            std::vector<Eigen::Vector3d> front_end_pos_yaw;
            void getGalaxConst(std::vector<Eigen::Vector3d> statelist);
            void getFIRIConst(std::vector<Eigen::Vector3d> statelist);

            void process(const ros::TimerEvent &);
            void warm(const ros::TimerEvent &);
            void targetCallback(const geometry_msgs::PoseStamped &msg);
            void odomCallback(const nav_msgs::OdometryPtr &msg);
            double pieceTime;
            /*ros related*/
            ros::Timer processTimer, cudaWarmTimer;
            ros::Subscriber targetSub, odomSub;
            ros::Publisher trajCmdPub;
            int startid = 10000, pathid = 1;




    };
}

#endif
