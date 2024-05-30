#ifndef CONFIG.HPP
#define CONFIG.HPP
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <Eigen/Eigen>
#include <XmlRpcValue.h>
struct Config
{
    double mini_T = 0.0;
    double wei_time_ = 500.0;
    double kmax = 0.45;
    double vmax = 3.0;
    double latAccmax = 3.0;
    double lonAccmax = 3.0;
    double accRatemax = 8.0;
    double kdotmax = 5.0;
    int traj_res = 16;
    double scaling_wf_min_ = 0.01, scaling_wc_min_ = 0.01;
    double rho = 1.0, rho_max =1000.0;
    double gamma = 1;
    double cons_eps_ = 0.20;
    int mem_size = 256;
    int past = 3; //3 
    double g_epsilon = 1.0e-3;
    double min_step = 1.0e-32;
    double delta = 1.0e-4;
    int max_iterations = 12000;
    int amlMaxIter = 15;
    double pieceTime = 0.7;
    XmlRpc::XmlRpcValue xmlconpts;
    XmlRpc::XmlRpcValue xmlconpts_astar;
    std::vector<double> start_vec;
    std::vector<double> end_vec;
    Eigen::Vector4d start_state; 
    Eigen::Vector4d end_state; 
    double FIRI_bound;
    std::vector<Eigen::Vector2d> conpts; 
    std::vector<Eigen::Vector2d> conpts_astar; 
    /*map parameters*/
    double mapRes = 0.1;
    double mapX = 50.0, mapY = 50.0, mapZ = 10.0;
    Eigen::Vector2d mapX_range{-1, 1};
    Eigen::Vector2d mapY_range{-1, 1};
    Eigen::Vector2d mapZ_range{-1, 1};
    std::vector<double> mapX_range_vec, mapY_range_vec, mapZ_range_vec;
    int expandSize = 1;
    /*front end*/
    double wheel_base = 0.6;
    double horizon_ = 50.0;
    double yaw_resolution_ = 0.3;
    double lambda_heu_ = 5.0;
    int allocate_num_ = 100000;
    int check_num_ = 5;
    double max_seach_time = 1.0;
    double step_arc = 0.9;
    double checkl = 0.2;
    double non_siguav = 0.0;
    double backW = 2.0;
    double penaWei = 500.0;
    double esdfWei = 1000.0;
    double miniS =0.9;
    bool isdebug = true;
    bool isfixGear = false;
    bool isVis = false;
    bool enable_shot = false;
    double safeMargin = 0.1;
    bool useDP = true;
    double phimax = 0.785;
    double omegamax = 0.3;
    double steer_res = 0.5;
    std::string model_path = "/home/han/2023codes/NeuralTraj/model/maze_res101.pt";
    std::string OutputPath_posyaw;
    bool once_vis_one = false;
    int once_duration_ms = 500;
    int skip_iter_duration_ms, skip_iter_vis;
    bool once_optimization;
    bool individual_vis = true;
    bool front_end_vis = true;
    double yawDotmax = 1.5;
    int plan_mode = 0;
    bool debug_front_end = false;
    std::vector<Eigen::Vector4d> goal_list;
    std::vector<double> goal_list_vec;


    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("yawDotmax", yawDotmax);
        nh_priv.param("mini_T", mini_T, 0.0);
        nh_priv.param("wei_time_", wei_time_, 500.0);
        nh_priv.param("kmax", kmax, 0.45);
        nh_priv.param("vmax", vmax, 3.0);
        nh_priv.param("latAccmax", latAccmax, 3.0);
        nh_priv.param("lonAccmax", lonAccmax, 3.0);
        nh_priv.param("accRatemax", accRatemax, 8.0);
        nh_priv.param("kdotmax", kdotmax, 1.0);
        nh_priv.param("traj_res", traj_res, 16);
        nh_priv.param("rho", rho, 1.0);
        nh_priv.param("rho_max", rho_max, 1000.0);
        nh_priv.param("gamma", gamma, 1.0);
        nh_priv.param("cons_eps_", cons_eps_, 0.2);
        nh_priv.param("mem_size", mem_size, 256);
        nh_priv.param("past", past, 3);
        nh_priv.param("g_epsilon", g_epsilon, 1.0e-3);
        nh_priv.param("min_step", min_step, 1.0e-32);
        nh_priv.param("delta", delta, 1.0e-4);
        nh_priv.param("max_iterations", max_iterations, 10000);
        nh_priv.param("amlMaxIter", amlMaxIter, 15);
        nh_priv.param("pieceTime", pieceTime, 0.7);
        nh_priv.param("FIRI_bound", FIRI_bound, 3.0);
        nh_priv.param("once_vis_one", once_vis_one, false);
        nh_priv.param("once_optimization", once_optimization, false);
        nh_priv.param("once_duration_ms", once_duration_ms, 500);
        nh_priv.param("skip_iter_vis", skip_iter_vis, 10);
        nh_priv.param("skip_iter_duration_ms", skip_iter_duration_ms, 500);
        nh_priv.param("front_end_vis", front_end_vis, false);
        nh_priv.param("plan_mode", plan_mode, 0);
        nh_priv.param("debug_front_end", debug_front_end, false);


        nh_priv.getParam("OutputPath_posyaw", OutputPath_posyaw);
        nh_priv.getParam("conpts", xmlconpts);
        nh_priv.getParam("conpts_astar", xmlconpts_astar);
        nh_priv.getParam("individual_vis", individual_vis);
        nh_priv.getParam("start_vec", start_vec);
        nh_priv.getParam("end_vec", end_vec);
        nh_priv.getParam("penaWei", penaWei);
        nh_priv.getParam("esdfWei", esdfWei);
        nh_priv.getParam("miniS", miniS);
        nh_priv.getParam("safeMargin", safeMargin);
        nh_priv.getParam("phimax", phimax);
        nh_priv.getParam("omegamax", omegamax);
        nh_priv.getParam("useDP", useDP);
        nh_priv.getParam("steer_res", steer_res);
        for(int i = 0; i < xmlconpts.size(); ++i)
        {
            Eigen::Vector2d pt;
            for(int j = 0; j < 2; ++j)
            {
                pt[j] = xmlconpts[i][j];
            }
            conpts.push_back(pt);
        }
        for(int i = 0; i < xmlconpts_astar.size(); ++i)
        {
            Eigen::Vector2d pt;
            for(int j = 0; j < 2; ++j)
            {
                pt[j] = xmlconpts_astar[i][j];
            }
            conpts_astar.push_back(pt);
        }
        start_state = Eigen::Vector4d{start_vec[0], start_vec[1], start_vec[2], start_vec[3]};
        end_state   = Eigen::Vector4d{  end_vec[0],   end_vec[1],   end_vec[2],   end_vec[3]};
        nh_priv.param("mapRes", mapRes, 0.1);
        nh_priv.param("mapX", mapX, 50.0);
        nh_priv.param("mapY", mapY, 50.0);
        nh_priv.param("expandSize", expandSize, 2);
        nh_priv.param("wheel_base", wheel_base, 0.6);
        nh_priv.param("horizon_", horizon_, 50.0);
        nh_priv.param("yaw_resolution_", yaw_resolution_, 0.3);
        nh_priv.param("lambda_heu_", lambda_heu_, 5.0);
        nh_priv.param("allocate_num_", allocate_num_, 100000);
        nh_priv.param("check_num_", check_num_, 5);
        nh_priv.param("max_seach_time", max_seach_time, 1.0);
        nh_priv.param("step_arc", step_arc, 0.9);
        nh_priv.param("checkl", checkl, 0.2);
        nh_priv.param("non_siguav", non_siguav, 0.0);
        nh_priv.param("isdebug", isdebug, false);
        nh_priv.param("isfixGear", isfixGear, false);
        nh_priv.param("isVis", isVis, false);
        nh_priv.param("enable_shot", enable_shot, false);
        nh_priv.param("backW", backW, 2.0);

        nh_priv.getParam("mapX_range", mapX_range_vec);
        nh_priv.getParam("mapY_range", mapY_range_vec);
        nh_priv.getParam("mapZ_range", mapZ_range_vec);
        if(mapX_range_vec.size() == 2)
            mapX_range = Eigen::Vector2d{mapX_range_vec[0], mapX_range_vec[1]};
        else
        {
            ROS_ERROR("Error in Param: mapX_range_vec");
            exit(0);
        }
        if(mapY_range_vec.size() == 2)
            mapY_range = Eigen::Vector2d{mapY_range_vec[0], mapY_range_vec[1]};
        else
        {
            ROS_ERROR("Error in Param: mapY_range_vec");
            exit(0);
        }
        if(mapZ_range_vec.size() == 2)
            mapZ_range = Eigen::Vector2d{mapZ_range_vec[0], mapZ_range_vec[1]};
        else if( mapZ_range_vec.size() == 0 )
            ROS_WARN("2D map Param");
        else
        {
            ROS_ERROR("Error in Param: mapZ_range_vec");
            exit(0);
        }

        nh_priv.getParam("goal_list", goal_list_vec);
        if( goal_list_vec.size() % 4 != 0 )
        {
            ROS_ERROR("Error in Param: goal_list");
            exit(0);
        }
        else
        {
            for( int ii = 0; ii < goal_list_vec.size(); ii += 4 )
            {
                goal_list.emplace_back(goal_list_vec[ii], goal_list_vec[ii+1], goal_list_vec[ii+2], goal_list_vec[ii+3]);
            }
        }
    }
};
double normalize_angle(const double& theta) {
    double theta_tmp = theta;    
    while(theta_tmp >= acos(-1.0)){
        theta_tmp -= 2 * acos(-1.0);
    }
    while(theta_tmp < -acos(-1.0)){
        theta_tmp += 2 * acos(-1.0);
    } 
  return theta_tmp;
}
typedef std::shared_ptr<Config> ConfigPtr;

#endif 