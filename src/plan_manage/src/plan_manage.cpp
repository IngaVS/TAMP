#include <plan_manage/plan_manage.h>
#include <tf/tf.h>
#include <tools/tic_toc.hpp>
#include <arcPlan/Trajopt_alm.hpp>
#include <difPlan/trajDif_opt.hpp>
// #include <arcPlan/Trajopt_penalty.hpp>
#include <fstream>
#include <random>
#include <thread>
using namespace plan_manage;
using namespace std;
void PlanManager::init(ros::NodeHandle& nh){
    vis_tool.reset(new visualization::Visualization(nh));
    config_.reset(new Config(nh));
    gridMap.setParam(config_, nh);
    //read map
    hasTarget = false;
    hasOdom = false;
    processTimer = nh.createTimer(ros::Duration(0.02), &PlanManager::process, this);
    cudaWarmTimer = nh.createTimer(ros::Duration(0.01), &PlanManager::warm,this);

    targetSub = nh.subscribe("/move_base_simple/goal", 1, &PlanManager::targetCallback, this);
    odomSub = nh.subscribe("/Odometry_map", 1, &PlanManager::odomCallback, this);
    vis_tool->registe<nav_msgs::Path>("/visualization/kinoPath");
    vis_tool->registe<nav_msgs::Path>("/visualization/AstarPath");
    vis_tool->registe<nav_msgs::Path>("/visualization/optTraj");
    vis_tool->registe<nav_msgs::Path>("/visualization/optTraj_kino");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/debugTraj");
    vis_tool->registe<nav_msgs::Path>("/visualization/NewTraj");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/waittoRefine");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/optArrowTraj");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/optArrowTraj2");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/start_goal");
    vis_tool->registe<visualization_msgs::Marker>("/visualization/goal");
    vis_tool->registe<decomp_ros_msgs::PolyhedronArray>("/visualization/sfc");
    vis_tool->registe<visualization_msgs::Marker>("/visualization/robot_once");  
    vis_tool->registe<visualization_msgs::Marker>("/visualization/robots_front_end");  
    vis_tool->registe<visualization_msgs::Marker>("/visualization/robots_all_traj");  
    vis_tool->registe<decomp_ros_msgs::PolyhedronArray>("/visualization/sfc_once");
    vis_tool->registe<nav_msgs::Path>("/visualization/refinedTraj2");
    vis_tool->registe<nav_msgs::Path>("/visualization/debugRefinedTraj2");
    vis_tool->registe<nav_msgs::Path>("/visualization/debugSe2Traj");
    vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/arrowTraj");

    vis_tool->registe<sensor_msgs::PointCloud2>("/visualization/wapoints_nn");
    vis_tool->registe<nav_msgs::Path>("/visualization/refinedTraj_nn");
    vis_tool->registe<visualization_msgs::Marker>("/visualization/fullshapeTraj_nn");

    vis_tool->registe<sensor_msgs::PointCloud2>("/visualization/wapoints_kinoastar");
    vis_tool->registe<nav_msgs::Path>("/visualization/refinedTraj_kinoastar");
    vis_tool->registe<nav_msgs::Path>("/visualization/refinedTraj_dftpav");
    vis_tool->registe<visualization_msgs::Marker>("/visualization/fullshapeTraj_kinoastar");


    pieceTime = config_->pieceTime;
    nh.param("data/mapid", startid, 1);
    nh.param("data/pathid", pathid, 1);

    
    kino_path_finder_.reset(new path_searching::KinoAstar);
    kino_path_finder_->init(config_, nh);  
    kino_path_finder_->intialMap(&gridMap);

    // Eigen::Vector2d map_XY{config_->mapX, config_->mapY};
    // interface_FIRI.setMap(0.5*map_XY, -0.5*map_XY);
    Eigen::Vector2d map_low_bound{config_->mapX_range[0], config_->mapY_range[0]};
    Eigen::Vector2d map_high_bound{config_->mapX_range[1], config_->mapY_range[1]};
    interface_FIRI.setMap(map_high_bound, map_low_bound);

    if(config_->useDP){
        trajCmdPub = nh.advertise<mpc_controller::DPtrajContainer>("/planner/dptrajectory", 1);  
    }
    else{
        trajCmdPub = nh.advertise<mpc_controller::PolyTraj>("/planner/trajectory", 1);  
    }

}
void PlanManager::warm(const ros::TimerEvent &){
    return;
}
void PlanManager::getGalaxConst(std::vector<Eigen::Vector3d> statelist){
    vec_Vec2f vec_obs = gridMap.getCloud();
    hPolys.clear();
    for(const auto state : statelist){
        Eigen::MatrixXd hPoly;
        Eigen::Vector2d pos = state.head(2);
        double yaw = state[2];
        std::vector<Eigen::Vector2d> add_vec_obs;
        Eigen::Matrix2d R;
        R << cos(yaw), -sin(yaw),
            sin(yaw),  cos(yaw);
        Eigen::Vector2d p;
        double d_x = 10.0;
        double d_y = 10.0;
        p = R*Eigen::Vector2d(d_x, d_y); 
        add_vec_obs.push_back(pos + p);
        p = R*Eigen::Vector2d(d_x, -d_y);
        add_vec_obs.push_back(pos + p);
        p = R*Eigen::Vector2d(-d_x, d_y);
        add_vec_obs.push_back(pos + p);
        p = R*Eigen::Vector2d(-d_x, -d_y);
        add_vec_obs.push_back(pos + p);
        plan_utils::corridorBuilder2d(pos, 100.0, 10.0, 10.0, vec_obs, add_vec_obs, hPoly);
        hPolys.push_back(hPoly);
    }
    return;
}


void PlanManager::getFIRIConst(std::vector<Eigen::Vector3d> statelist){
    robots.clear();
    hPolys.clear();
    std::vector<Eigen::Vector2d> obstacle;

    Eigen::Matrix2Xd robot_pc;
    robot_pc.resize(2, config_->conpts.size());
    for( int i = 0; i < config_->conpts.size(); i++ )
        robot_pc.col(i) = config_->conpts[i];

    TicToc timer;
    for(const auto state : statelist){
        front_end_pos_yaw.push_back(state);
        obstacle = gridMap.getInputCloud(state, config_->FIRI_bound);        
        // std::cout << "obstacle size: " << obstacle.size() << std::endl;

        Eigen::MatrixXd hPoly;
        Eigen::Vector2d pos = state.head(2);
        double yaw = state[2];
        Eigen::Matrix2d R;
        R << cos(yaw), -sin(yaw),
            sin(yaw),  cos(yaw);
        Eigen::Matrix2Xd robot_real = (R * robot_pc).colwise() + pos;
        double run_time;
        interface_FIRI.quickRun(pos, robot_real, obstacle, config_->FIRI_bound, hPoly, run_time);

        robots.push_back(robot_real);
        hPolys.push_back(hPoly);
    }
    std::cout << "FIRI time: " << timer.toc() << " ms" << std::endl;
    std::cout << "statelist size: " << statelist.size() << std::endl;

    return;
}

// void PlanManager::getFIRIConst(std::vector<Eigen::Vector3d> statelist){
//     TicToc timer;
//     vec_Vec2f vec_obs = gridMap.getCloud();
//     // vec_Vec2f vec_obs = gridMap.getInputCloud();
//     std::vector<Eigen::Vector2d> obstacle;
//     obstacle.resize(vec_obs.size());
//     for( int i = 0; i < vec_obs.size(); i++ )
//         obstacle[i] = vec_obs[i];
//     robots.clear();
//     hPolys.clear();

//     Eigen::Matrix2Xd robot_pc;
//     robot_pc.resize(2, config_->conpts.size());
//     for( int i = 0; i < config_->conpts.size(); i++ )
//         robot_pc.col(i) = config_->conpts[i];

//     std::cout << "getPC time: " << timer.toc() << " ms" << std::endl;

//     timer.tic();
//     for(const auto state : statelist){
//         Eigen::MatrixXd hPoly;
//         Eigen::Vector2d pos = state.head(2);
//         double yaw = state[2];
//         Eigen::Matrix2d R;
//         R << cos(yaw), -sin(yaw),
//             sin(yaw),  cos(yaw);
//         Eigen::Matrix2Xd robot_real = (R * robot_pc).colwise() + pos;
//         double run_time;
//         interface_FIRI.quickRun(pos, robot_real, obstacle, config_->FIRI_bound, hPoly, run_time);

//         robots.push_back(robot_real);
//         hPolys.push_back(hPoly);
//     }
//     std::cout << "FIRI time: " << timer.toc() << " ms" << std::endl;
//     return;
// }

void PlanManager::odomCallback(const nav_msgs::OdometryPtr &msg)
{
    odom[0] = msg->pose.pose.position.x;
    odom[1] = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    odom[2] = atan2(R.col(0)[1],R.col(0)[0]);
    hasOdom = true;
    return;
}
void PlanManager::targetCallback(const geometry_msgs::PoseStamped &msg){
    ROS_INFO("Recieved target!");
    targetPose <<  msg.pose.position.x, msg.pose.position.y,
                tf::getYaw(msg.pose.orientation);
    // targetPose << (6.0+5.3) / 2, 5.5, -M_PI / 2.0;
    // targetPose << 11.701,  4.11058, -1.84895;
    std::cout<<"targetPose: "<<targetPose.transpose()<<std::endl;
    targetPose_vec.push_back(targetPose);
    if(config_->plan_mode == 2) 
    {
        if( targetPose_vec.size() != 2 )
        {
            ROS_WARN("Wait Goal!!!");
            ROS_WARN("Wait Goal!!!");
            ROS_WARN("Wait Goal!!!");
            ROS_WARN("Wait Goal!!!");
            ROS_WARN("Wait Goal!!!");
        }
    }

    hasTarget = true;
    return;
}
void PlanManager::process(const ros::TimerEvent &){
    if(!gridMap.has_map_()) return;
    // neural_path_finder_->intialMap(&gridMap);
    gridMap.publishPCL();
    // std::cout << "pcl number: "<<gridMap.getCloud().size()<<std::endl;
    // if(!hasTarget || !hasOdom) return;
    if(!hasTarget) return;
    if(config_->plan_mode == 2) 
    {
        if( targetPose_vec.size() != 2 )
        {
            return;
        }
    }

    if( !first_time )
    {
        first_time = true;
        first_odom = odom;
    }
    ROS_WARN("Triggering------------------------------------ we begin to plan a trajectory!");
    hasTarget = false;


    vector<Eigen::Vector3d> sampleTraj;
    path_searching::KinoTrajData kino_trajs_;
    std::vector<Eigen::Vector3d> visKinoPath;
    kino_path_finder_->reset();
    Eigen::Vector4d iniFs, finFs;
    iniFs = config_->start_state;
    finFs = config_->end_state;

    // 来回起点终点
    // if(config_->plan_mode == 0) 
    // {
    //     if(plan_id % 2 == 0)
    //     {
    //         iniFs = config_->start_state;
    //         iniFs[0] = odom[0];
    //         iniFs[1] = odom[1];
    //         finFs = config_->end_state;
    //     }
    //     else
    //     {
    //         iniFs = config_->end_state;
    //         iniFs[0] = odom[0];
    //         iniFs[1] = odom[1];
    //         finFs = config_->start_state;
    //     }
    //     plan_id++;
    // }
    // // 点goal
    // else if(config_->plan_mode == 1) 
    // {
    //     iniFs[0] = odom[0];
    //     iniFs[1] = odom[1];
    //     iniFs[2] = odom[2];
    //     iniFs[3] = 0.0;
    //     finFs[0] = targetPose[0];
    //     finFs[1] = targetPose[1];
    //     finFs[2] = targetPose[2];
    //     finFs[3] = 0.0;

    //     if( goal_num == 4 )
    //     {
    //         finFs[0] = first_odom[0];
    //         finFs[1] = first_odom[1];
    //         finFs[2] = first_odom[2];
    //     }
    // }
    // else if(config_->plan_mode == 2) 
    // {
    //     iniFs[0] = targetPose_vec[0][0];
    //     iniFs[1] = targetPose_vec[0][1];
    //     iniFs[2] = targetPose_vec[0][2];
    //     iniFs[3] = 0.0;
    //     finFs[0] = targetPose_vec[1][0];
    //     finFs[1] = targetPose_vec[1][1];
    //     finFs[2] = targetPose_vec[1][2];
    //     finFs[3] = 0.0;
    //     targetPose_vec.clear();
    //     ROS_WARN("Wait Start!!!");
    //     ROS_WARN("Wait Start!!!");
    //     ROS_WARN("Wait Start!!!");
    //     ROS_WARN("Wait Start!!!");
    //     ROS_WARN("Wait Start!!!");
    // }
    // else if(config_->plan_mode == 3) 
    // {
    //     if(targetPose_vec.size() < 2)
    //     {
    //         ROS_WARN("Do Not Have Enuogh Target!!!");
    //         ROS_WARN("Do Not Have Enuogh Target!!!");
    //         ROS_WARN("Do Not Have Enuogh Target!!!");
    //         ROS_WARN("Do Not Have Enuogh Target!!!");
    //         ROS_WARN("Do Not Have Enuogh Target!!!");
    //         return;
    //     }
    //     int targets_size = targetPose_vec.size();
    //     iniFs[0] = targetPose_vec[targets_size-2][0];
    //     iniFs[1] = targetPose_vec[targets_size-2][1];
    //     iniFs[2] = targetPose_vec[targets_size-2][2];
    //     iniFs[3] = 0.0;
    //     finFs[0] = targetPose_vec[targets_size-1][0];
    //     finFs[1] = targetPose_vec[targets_size-1][1];
    //     finFs[2] = targetPose_vec[targets_size-1][2];
    //     finFs[3] = 0.0;
    // }
    // else if(config_->plan_mode == 4) 
    // {
    //     if(goal_list_id == config_->goal_list.size()-1)
    //     {
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         return;
    //     }
    //     iniFs = config_->goal_list[goal_list_id];
    //     finFs = config_->goal_list[goal_list_id+1];
    //     goal_list_id++;
    // }
    // else if(config_->plan_mode == 5) 
    // {
    //     if(goal_list_id == config_->goal_list.size()-1)
    //     {
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         ROS_WARN("All Goal is Pass!!!");
    //         return;
    //     }
    //     iniFs = config_->goal_list[goal_list_id];
    //     finFs = config_->goal_list[goal_list_id+1];
    //     if( goal_list_id == 0 )
    //     {
    //         iniFs[0] = odom[0];
    //         iniFs[1] = odom[1];
    //         iniFs[2] = odom[2];
    //     }
    //     if( goal_list_id + 1 == config_->goal_list.size()-1 )
    //     {
    //         finFs[0] = first_odom[0];
    //         finFs[1] = first_odom[1];
    //         finFs[2] = first_odom[2];
    //     }
    //     std::vector<Eigen::Vector3d> arrows;
    //     arrows.resize(2);
    //     arrows[0] = finFs.head<3>();
    //     arrows[0][2] = 0;

    //     arrows[1] = finFs.head<3>();
    //     arrows[1][0] += 1.5 * cos(finFs[2]);
    //     arrows[1][1] += 1.5 * sin(finFs[2]);
    //     arrows[1][2] = 0;

    //     vis_tool->visualize_arrow(arrows, "/visualization/goal");

    //     goal_list_id++;
    // }
    // else
    // {
    //     ROS_ERROR("Error plan mode!!!");
    //     exit(0);
    // }


    iniFs = config_->start_state;
    finFs[0] = targetPose[0];
    finFs[1] = targetPose[1];
    finFs[2] = targetPose[2];
    finFs[3] = 0.0;

    std::vector<Eigen::Vector4d> start_goal;
    start_goal.push_back(iniFs);
    start_goal.push_back(finFs);
    vis_tool->visualize_balls_wxx(start_goal, "/visualization/start_goal");
    // iniFs <<  0,9.5,-M_PI_2, 0.0;
    // finFs << 9.5,0,0, 0.0;
    TicToc time_profile_tool_;
    time_profile_tool_.tic();
    double model_time;
    double t1 = ros::Time::now().toSec();
    int status = kino_path_finder_->search(iniFs, Eigen::Vector2d::Zero(), finFs, model_time, true);
    if(status == 0)
    {
        ROS_ERROR("Bad Goal!!!");
        ROS_ERROR("Please Try Another Goal!!!");
        return;
    }
    goal_num++;
    double t2 = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> ts = kino_path_finder_->getRoughSamples();
    sampleTraj.insert(sampleTraj.end(), ts.begin(), ts.end());
    

    kino_path_finder_->getKinoNode(kino_trajs_, sampleTraj);
    // kino_path_finder_->getKinoNode(kino_trajs_);
    for(int i = 0; i < sampleTraj.size(); i++){
        Eigen::Vector3d pos;
        pos.head(2) = sampleTraj[i].head(2);
        pos[2] = 0.2;
        visKinoPath.push_back(pos);
    }
    vis_tool->visualize_path(visKinoPath, "/visualization/kinoPath");
    TicToc Corridor_timer;
    std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
    int segnum = kino_trajs_.size();    
    std::vector<int> refined_singuals; refined_singuals.resize(segnum);
    Eigen::VectorXd refined_rt;
    refined_rt.resize(segnum);
    std::vector<Eigen::MatrixXd> refined_inPs_container;
    refined_inPs_container.resize(segnum);
    std::vector<Eigen::Vector2d> refined_gearPos;
    std::vector<double> refined_angles; 
    refined_gearPos.resize(segnum - 1); refined_angles.resize(segnum - 1);
    double basetime = 0.0;
    std::vector<int> pnums;
    std::vector<Eigen::MatrixXd> hPolys_vis;
    std::vector<Eigen::Matrix2Xd> robots_vis;
    std::vector<double> time_for_vis;
    hPolys_vis.clear();
    front_end_pos_yaw.clear();
    for(int i = 0; i < segnum; i++){
        double timePerPiece = pieceTime;
        path_searching::FlatTrajData kino_traj = kino_trajs_.at(i);
        refined_singuals[i] = kino_traj.singul;
        std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
        int piece_nums;
        double initTotalduration = 0.0;
        for(const auto pt : pts){
            initTotalduration += pt[2];
        }
        piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),1);
        double dt = initTotalduration / piece_nums; 
        refined_rt[i] = (initTotalduration / piece_nums);
        pnums.push_back(piece_nums);

        refined_inPs_container[i].resize(2, piece_nums - 1);
        for(int j = 0; j < piece_nums - 1; j++ ){
            double t = basetime + (j+1)*dt;
            Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
            refined_inPs_container[i].col(j) = pos.head(2);
        }
        if(i >=1){
            Eigen::Vector3d pos = kino_path_finder_->evaluatePos(basetime);
            refined_gearPos[i-1] = pos.head(2);
            refined_angles[i-1] = pos[2];
        }

        std::vector<Eigen::Vector3d> statelist;
        double res_time = 0;
        for(int i = 0; i < piece_nums; i++ ){
            int resolution = config_->traj_res;
            for(int k = 0; k <= resolution; k++){
                double t = basetime+res_time + 1.0*k/resolution*dt;
                Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
                statelist.push_back(pos);
                time_for_vis.push_back(t);
                }
            res_time += dt;
        }

        getFIRIConst(statelist);//here
        // getGalaxConst(statelist);//here
        sfc_container.push_back(hPolys);//here
        for( int ii = 0; ii < hPolys.size(); ii++ )
        {
            hPolys_vis.push_back(hPolys[ii]);
            robots_vis.push_back(robots[ii]);
        }

        basetime += initTotalduration;
    }
    std::ofstream file(config_->OutputPath_posyaw, std::ios::trunc);
    if (file.is_open()) 
    {
        file << "x" << "," << "y" << "," << "yaw";
        file << "\n";

        for( int ii = 0; ii < front_end_pos_yaw.size(); ii++ )
        {
            file << front_end_pos_yaw[ii][0] << "," << front_end_pos_yaw[ii][1] << "," << front_end_pos_yaw[ii][2];
            file << "\n";
        }
        file.close();
        std::cout << "Successfully written to " << config_->OutputPath_posyaw << std::endl;
    } 
    else 
    {
        std::cout << "Unable to open file " << config_->OutputPath_posyaw << std::endl;
    }

    // 可视化前端
    if( config_->front_end_vis )
    {
        vis_tool->visualize_rectangles(robots_vis, "/visualization/robots_front_end");
    }
    // 可视化单个凸包
    //! rosparam set /input_id 0
    if( config_->individual_vis )
    {
        int last_id, input_id = 0;
        while(true)
        {
            ros::param::get("/input_id", input_id);
            if( last_id == input_id )
            {
                std::chrono::milliseconds waitTime(500);
                std::this_thread::sleep_for(waitTime);
                continue;
            }
            else
            {
                last_id = input_id;
            }
            
            std::cout << "input id: " << input_id << std::endl;
            if( input_id >= 0 && input_id < (int)hPolys_vis.size() )
            {
                std::cout << "vis the " << input_id << "-th convex polytope" << std::endl;
                vis_tool->visualize_sfc(std::vector<Eigen::MatrixXd>{hPolys_vis[input_id]}, "/visualization/sfc_once");
                vis_tool->visualize_rectangle(robots_vis[input_id], "/visualization/robot_once");
            }
            else if( input_id >= (int)hPolys_vis.size() )
            {
                std::cout << "the input is too big!" << std::endl;
            }
            else
            {
                std::cout << "the input < 0. break!" << std::endl;
                break;
            }
        }        
    }

    vis_tool->visualize_sfc(hPolys_vis, "/visualization/sfc");
    std::cout << "\033[1;32m" << "Corridor_timer: " << Corridor_timer.toc() << " ms" << "\033[0m" << std::endl;

    if( config_->debug_front_end )
        return;

    std::chrono::milliseconds waitTime_opt(2000);
    std::this_thread::sleep_for(waitTime_opt);

    iniState2d << iniFs[0], refined_singuals[0] * cos(iniFs[2]), 0.0,
                iniFs[1], refined_singuals[0] * sin(iniFs[2]), 0.0;
    finState2d << finFs[0], refined_singuals[segnum-1] * cos(finFs[2]), 0.0,
                finFs[1], refined_singuals[segnum-1] * sin(finFs[2]), 0.0;
    // ROS_INFO("begin to refine");
    // PolyTrajOpt::TrajOpt refinedOpt;

    PolyTrajOpt::TrajOpt refinedOpt;
    // 25ms
    refinedOpt.setCors(sfc_container);//here

    double t3 = ros::Time::now().toSec();
    ROS_ERROR("1111111111111111111111");
    int flagSs = refinedOpt.OptimizeSe2Trajectory(
    iniState2d, finState2d, refined_rt,
    refined_inPs_container, refined_gearPos,
    refined_angles, &gridMap,  config_, refined_singuals, vis_tool,"kinoastar");
    PolyTrajOpt::UgvTrajectory optTraj = refinedOpt.getOptTraj();
    ROS_WARN_STREAM("dptraj total arc: "<<optTraj.getTotalArc());
    ROS_WARN_STREAM("dptraj traj time: "<<optTraj.getTotalDuration());
    std::vector<Eigen::Matrix2Xd> robots_vis_backend = refinedOpt.robots_vis;
    for( int ii = 0; ii < robots_vis_backend.size(); ii++ )
    {
        robots_vis_all.push_back(robots_vis_backend[ii]);
    }
    vis_tool->visualize_rectangles(robots_vis_all, "/visualization/robots_all_traj", 1.0, 0.03);


    {
        mpc_controller::DPtrajContainer trajmsg;
        for(int i = 0; i < optTraj.getSegNum(); i++){
            int singual = optTraj.etas[i];
            mpc_controller::PolyTrajAC trajSegment;
            for(int j = 0; j < optTraj.Traj_container[i].getPieceNum(); j++){
                mpc_controller::SinglePolyAC piece;
                piece.dt = optTraj.Traj_container[i].tTraj[j].getDuration();
                piece.ds = optTraj.Traj_container[i].posTraj[j].getDuration();

                Eigen::Matrix<double, 2, 6> c = optTraj.Traj_container[i].posTraj[j].getCoeffMat();
                Eigen::Matrix<double, 1, 6> c1 = optTraj.Traj_container[i].tTraj[j].getCoeffMat(); 
                for (int k=0; k<6; k++)
                {
                    piece.coef_x.push_back(c(0, k));
                    piece.coef_y.push_back(c(1, k));
                    piece.coef_s.push_back(c1(0, k));
                }
                trajSegment.trajs.push_back(piece);
            }
            trajmsg.traj_container.push_back(trajSegment);
            // trajmsg.reverse
            if(singual > 0) trajmsg.reverse.push_back(false);
            else trajmsg.reverse.push_back(true);
        }
        trajmsg.start_time = ros::Time::now();
        trajCmdPub.publish(trajmsg);
    }

    // std::chrono::milliseconds waitTime_car(60000);
    // std::this_thread::sleep_for(waitTime_car);


    // 按顺序一个一个可视化凸包
    if( config_->once_vis_one )
    {
        std::chrono::milliseconds waitTime_once(2000);
        std::this_thread::sleep_for(waitTime_once);
        std::vector<Eigen::MatrixXd> hPolys_vis_once;
        hPolys_vis_once.clear();
        double last_time = 0;
        vis_tool->visualize_rectangles(robots_vis, "/visualization/robots_front_end");
        for( int ii = 0; ii < hPolys_vis.size(); ii++ )
        {
            hPolys_vis_once.push_back(hPolys_vis[ii]);
            vis_tool->visualize_sfc(std::vector<Eigen::MatrixXd>{hPolys_vis[ii]}, "/visualization/sfc_once");
            std::chrono::milliseconds waitTime_try(20);
            std::this_thread::sleep_for(waitTime_try);
            vis_tool->visualize_rectangle(robots_vis[ii], "/visualization/robot_once");
            vis_tool->visualize_sfc(hPolys_vis_once, "/visualization/sfc");
            
            if( config_->once_duration_ms < 0 )
            {
                double wait_time = time_for_vis[ii] - last_time;
                last_time = time_for_vis[ii];
                int wait_time_ms = wait_time * 1000 * (-config_->once_duration_ms);
                std::chrono::milliseconds waitTime(wait_time_ms);
                std::this_thread::sleep_for(waitTime);
            }
            else
            {
                std::chrono::milliseconds waitTime(config_->once_duration_ms);
                std::this_thread::sleep_for(waitTime);
            }
        }
        hPolys_vis_once.clear();
        vis_tool->visualize_sfc(hPolys_vis_once, "/visualization/sfc_once");
    }
}
