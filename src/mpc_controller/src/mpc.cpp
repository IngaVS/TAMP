#include "mpc/mpc.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
using namespace std;
 
void MPC::init(ros::NodeHandle &nh)
{
    nh.param("mpc/du_threshold", du_th, -1.0);
    nh.param("mpc/dt", dt, -1.0);
    nh.param("mpc/max_iter", max_iter, -1);
    nh.param("mpc/predict_steps", T, -1);
    nh.param("mpc/max_omega", max_omega, -1.0);
    nh.param("mpc/max_domega", max_domega, -1.0);
    nh.param("mpc/max_speed", max_speed, -1.0);
    nh.param("mpc/min_speed", min_speed, -1.0);
    nh.param("mpc/max_accel", max_accel, -1.0);
    nh.param("mpc/delay_num_v", delay_num_v, -1);
    nh.param("mpc/delay_num_w", delay_num_w, -1);
    nh.param("mpc/test_mpc", test_mpc, false);
    nh.param("mpc/model_type", model_type, DIFF);
    nh.param<std::vector<double>>("mpc/matrix_q", Q, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_r", R, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_rd", Rd, std::vector<double>());

    cout<<"dt="<<dt<<endl;

    has_odom = false;
    receive_traj = false;
    max_comega = max_domega * dt;
    max_cv = max_accel * dt;
    xref = Eigen::Matrix<double, 3, 500>::Zero(3, 500);
    last_output = output = dref = Eigen::Matrix<double, 2, 500>::Zero(2, 500);
    max_delay_num = max(delay_num_v, delay_num_w);
    delay_num_Asub = delay_num_v>delay_num_w?(delay_num_v-delay_num_w)*2:(delay_num_w-delay_num_v);
    for (int i=0; i<max_delay_num; i++)
        output_buff.push_back(Eigen::Vector2d::Zero());
    // cmd.height = 1.0;
    // cmd.height_vel = 0.0;
    // cmd.omega = 0.0;
    // cmd.pitch_vel = 0.0;
    // cmd.roll = 0.0;
    // cmd.speed = 0.0;

    pos_cmd_pub_ = nh.advertise<diablo_sdk::Diablo_Ctrl>("cmd", 200);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    vis_pub = nh.advertise<visualization_msgs::Marker>("/following_path", 10);
    predict_pub = nh.advertise<visualization_msgs::Marker>("/predict_path", 10);
    error_pub = nh.advertise<std_msgs::Float32>("tk_error",10);
        px_pub_ = nh.advertise<std_msgs::Float32>("/px_pub_",10);
            py_pub_ = nh.advertise<std_msgs::Float32>("/py_pub_",10);
                yaw_pub_ = nh.advertise<std_msgs::Float32>("/yaw_pub_",10);
                    vel_pub_ = nh.advertise<std_msgs::Float32>("vel_pub_",10);
    ref_pub = nh.advertise<visualization_msgs::Marker>("/ref_point", 10);
    test_pub = nh.advertise<nav_msgs::Path>("/test_path", 10);

    cmd_timer_ = nh.createTimer(ros::Duration(0.01), &MPC::cmdCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &MPC::rcvOdomCallBack, this, ros::TransportHints().tcpNoDelay());
    // odom_sub_ = nh.subscribe("odom", 1, &MPC::rcvOdomCallBack, this);
    
    trajopt.init(nh);
    
    if (test_mpc)
    {
        trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MPC::rcvTriggerCallBack, this);
    }
    else
    {
        traj_sub_ = nh.subscribe("/planner/dptrajectory", 1, &MPC::rcvDPCallBack, this);
        // traj_sub_ = nh.subscribe("traj", 1, &MPC::rcvTrajCallBack, this);
    }
}

void MPC::rcvTriggerCallBack(const geometry_msgs::PoseStamped msg)
{
    receive_traj = true;
    double r = 2.5;
    Eigen::MatrixXd eight(9, 2);
    eight << initPos[0]+  r,initPos[1]+r,
             initPos[0]+2*r,initPos[1]+2*r,
             initPos[0]+3*r,initPos[1]+r,
             initPos[0]+4*r,initPos[1]+0,
             initPos[0]+5*r,initPos[1]+r,
             initPos[0]+4*r,initPos[1]+2*r,
             initPos[0]+3*r,initPos[1]+r,
             initPos[0]+2*r,initPos[1]+0,
             initPos[0]+r  ,initPos[1]+r;
    

      Eigen::MatrixXd posP = eight.transpose();
      Eigen::VectorXd T;T.resize(1);T[0]=r;
      Eigen::MatrixXd initS, tailS;
      initS.setZero(2, 3);
      tailS.setZero(2, 3);
      initS << initPos[0],1.0,0.0,
               initPos[1],0.0,0.0;
      tailS << initPos[0]+0.0,-1.0,0.0,
               initPos[1]+2*r,0,0.0;
    
        

    std::vector<Eigen::MatrixXd> ps; ps.push_back(posP);
    trajopt.OptimizeSe2Trajectory(initS,tailS,0.0,0.0,T,ps);
    dptraj_mpc::UgvTrajectory tj = trajopt.getOptTraj();
    tj.startTime = ros::Time::now().toSec();
    traj_analyzer.setTraj(tj);

    std::vector<Eigen::Vector3d> poses = trajopt.getTraj(0.1);
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "world";
    for (const auto& pt : poses) {
        tmpPose.pose.position.x = pt[0];
        tmpPose.pose.position.y = pt[1];
        tmpPose.pose.position.z = 0.2;
        path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    test_pub.publish(path_msg);

}

void MPC::rcvTrajCallBack(mpc_controller::PolynomeConstPtr msg)
{
    ROS_WARN("222222222222222");
    receive_traj = true;
    traj_analyzer.setTraj(msg);
}

void MPC::rcvMincoCallBack(mpc_controller::MincoTrajConstPtr msg)
{
    ROS_WARN("000000000000000000000000");
    receive_traj = true;
    traj_analyzer.setTraj(msg);
}
void MPC::rcvDPCallBack(mpc_controller::DPtrajContainerConstPtr msg)
{
    ROS_WARN("111111111111111111111111111");
    receive_traj = true;
    traj_analyzer.setTraj(msg);
}

void MPC::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
    // ROS_WARN("ODOM");
    if(!has_odom){
        initPos <<msg->pose.pose.position.x,msg->pose.pose.position.y;
    }
    has_odom = true;
    now_state.x = msg->pose.pose.position.x;
    now_state.y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
    now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
    now_input.vx = lvel.norm();

    double direction = atan2(lvel(1), lvel(0));
    if ((direction-now_state.theta)>M_PI/2)
    {
        now_input.vx = -now_input.vx;
    }
}

void MPC::cmdCallback(const ros::TimerEvent &e)
{
    drawFollowPath();
    if (!has_odom || !receive_traj)
        return;

    // ROS_WARN("????");


    vector<TrajPoint> P = traj_analyzer.getRefPoints(T, dt);
    if (traj_analyzer.at_goal)
    {
        cmd.speed = 0.0;
        cmd.omega = 0.0;
    }
    else
    {
        Eigen::Vector2d tkerror; tkerror << now_state.x-P[0].x,now_state.y-P[0].y;
        std_msgs::Float32 tkmsg;
        tkmsg.data = tkerror.norm();
        error_pub.publish(tkmsg);
        for (int i=0; i<T; i++)
        {
            xref(0, i) = P[i].x;
            xref(1, i) = P[i].y;
            xref(2, i) = P[i].theta;
            dref(0, i) = P[i].v;
            dref(1, i) = 0.0;
        }
        smooth_yaw();
        getCmd();


        
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = cmd.speed;
    cmd_vel.angular.z = cmd.omega;
    cmd_pub_.publish(cmd_vel);

    pos_cmd_pub_.publish(cmd);
}

void MPC::getLinearModel(const MPCNode& node)
{
    B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = cos(node.first.theta) * dt;
    B(1, 0) = sin(node.first.theta) * dt;
    B(2, 1) = dt;

    A = Eigen::Matrix3d::Identity();
    A(0, 2) = -B(1, 0) * node.second.vx;
    A(1, 2) = B(0, 0) * node.second.vx;

    C = Eigen::Vector3d::Zero();
    C(0) = -A(0, 2) * node.first.theta; 
    C(1) = -A(1, 2) * node.first.theta;     
}

void MPC::stateTrans(MPCNode& node)
{
    node.second.vx = max(min(node.second.vx, max_speed), min_speed);
    node.second.w = max(min(node.second.w, max_omega), -max_omega);

    node.first.x = node.first.x + node.second.vx * cos(node.first.theta) * dt;
    node.first.y = node.first.y + node.second.vx * sin(node.first.theta) * dt;

    node.first.theta = node.first.theta + node.second.w * dt;
}

void MPC::predictMotion(void)
{
    xbar[0].first = now_state;

    MPCState temp = now_state;
    for (int i=1; i<T+1; i++)
    {
        xbar[i-1].second.vx = output(0, i-1);
        xbar[i-1].second.w = output(1, i-1);
        xbar[i-1].second.delta = output(1, i-1);
        xbar[i] = xbar[i-1];
        stateTrans(xbar[i]);
    }
}

void MPC::predictMotion(MPCState* b)
{
    b[0] = xbar[0].first;

    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;
    Eigen::MatrixXd Cx;
    Eigen::MatrixXd xnext;
    MPCState temp = xbar[0].first;
         
    for (int i=1; i<T+1; i++)
    {  
        MPCState temp_i = xbar[i-1].first;
        
        Bx = Eigen::Matrix<double, 3, 2>::Zero();
        Bx(0, 0) = cos(temp_i.theta) * dt;
        Bx(1, 0) = sin(temp_i.theta) * dt;
        Bx(2, 1) = dt;
        
        Ax = Eigen::Matrix3d::Identity();
        Ax(0, 2) = -Bx(1, 0) * xbar[i-1].second.vx;
        Ax(1, 2) = Bx(0, 0) * xbar[i-1].second.vx;

        Cx = Eigen::Vector3d::Zero();
        Cx(0) = -Ax(0, 2) * temp_i.theta;
        Cx(1) = -Ax(1, 2) * temp_i.theta;
        xnext = Ax*Eigen::Vector3d(temp.x, temp.y, temp.theta) + Bx*Eigen::Vector2d(output(0, i-1), output(1, i-1)) + Cx;
        temp.x = xnext(0);
        temp.y = xnext(1);
        temp.theta = xnext(2);
        b[i] = temp;
    }

}

void MPC::solveMPCDiff(void)
{
    const int delay_num_min = min(delay_num_v, delay_num_w);
    const int dimx = 3 * (T - delay_num_min);
    const int dimv = T - delay_num_v;
    const int dimw = T - delay_num_w;
    const int dimu = dimv + dimw;
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=delay_num_min; i<dimx; i+=3, j++)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    const int nnzQ_w = nx + dimv - 1;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
    }
    double sub_temp_v = Rd[0]*2.0;
    double sub_temp_w = Rd[1]*2.0;
    for (int i=dimx; i<dimx+dimv; i++)
    {
        dQ[i] = 2 * (R[0] + sub_temp_v);
    }
    for (int i=dimx+dimv; i<nx; i++)
    {
        dQ[i] = 2 * (R[1] + sub_temp_w);
    }
    dQ[dimx] -= sub_temp_v;
    dQ[dimx+dimv-1] -= sub_temp_v;
    dQ[dimx+dimv] -= sub_temp_w;
    dQ[nx-1] -= sub_temp_w;

    for (int i=nx; i<nnzQ_w; i++)
    {
        irowQ[i] = i - dimu + 1;
        jcolQ[i] = i - dimu;
        dQ[i]    = -Rd[0] * 2.0;
    }
    for (int i=nnzQ_w; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu + 1;
        dQ[i]    = -Rd[1] * 2.0;
    }

    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCNode temp = xbar[delay_num_min];
    getLinearModel(temp);
    int my = dimx;
    double b[my];
    const int nnzA = 11 * (T-delay_num_min) - 5 - delay_num_Asub;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];

    Eigen::Vector3d temp_vec(temp.first.x, temp.first.y, temp.first.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    if (delay_num_v > delay_num_w)
    {
        b[0] += temp.second.vx * B(0, 0);
        b[1] += temp.second.vx * B(1, 0);

        irowA[dimx] = 2;
        jcolA[dimx] = dimx+dimv;
        dA[dimx] = -B(2, 1);

        int ABsbegin = dimx+1;
        for (int i=0, j=1; j<delay_num_v-delay_num_w; i+=6, j++)
        {
            getLinearModel(xbar[j+delay_num_min]);
            temp_b = C;
            temp_b[0] += xbar[j+delay_num_min].second.vx * B(0, 0);
            temp_b[1] += xbar[j+delay_num_min].second.vx * B(1, 0);

            for (int k=0; k<3; k++)
            {
                b[3*j+k] = temp_b[k];
                irowA[ABsbegin + i + k] = 3*j + k;
                jcolA[ABsbegin + i + k] = irowA[ABsbegin + i + k] - 3;
                dA[ABsbegin + i + k] = -A(k, k);
            }
            irowA[ABsbegin + i + 3] = 3*j;
            jcolA[ABsbegin + i + 3] = 3*j - 1;
            dA[ABsbegin + i + 3] = -A(0, 2);

            irowA[ABsbegin + i + 4] = 3*j + 1;
            jcolA[ABsbegin + i + 4] = 3*j - 1;
            dA[ABsbegin + i + 4] = -A(1, 2);

            irowA[ABsbegin + i + 5] = 3*j + 2;
            jcolA[ABsbegin + i + 5] = dimx + dimv + j;
            dA[ABsbegin + i + 5] = -B(2, 1);
        }

        int ABbegin = ABsbegin + (delay_num_v-delay_num_w-1)*6;
        int ABidx = 8*(T-max_delay_num);
        for (int i=0, j=delay_num_v-delay_num_w; i<ABidx; i+=8, j++)
        {
            getLinearModel(xbar[j+delay_num_min]);

            for (int k=0; k<3; k++)
            {
                b[3*j+k] = C[k];
                irowA[ABbegin + i + k] = 3*j + k;
                jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
                dA[ABbegin + i + k] = -A(k, k);
            }
            irowA[ABbegin + i + 3] = 3*j;
            jcolA[ABbegin + i + 3] = 3*j - 1;
            dA[ABbegin + i + 3] = -A(0, 2);

            irowA[ABbegin + i + 4] = 3*j + 1;
            jcolA[ABbegin + i + 4] = 3*j - 1;
            dA[ABbegin + i + 4] = -A(1, 2);
            
            irowA[ABbegin + i + 5] = 3*j;
            jcolA[ABbegin + i + 5] = dimx + j - (delay_num_v-delay_num_w);
            dA[ABbegin + i + 5] = -B(0, 0);
            
            irowA[ABbegin + i + 6] = 3*j + 1;
            jcolA[ABbegin + i + 6] = jcolA[ABbegin + i + 5];
            dA[ABbegin + i + 6] = -B(1, 0);
            
            irowA[ABbegin + i + 7] = 3*j + 2;
            jcolA[ABbegin + i + 7] = dimx + dimv + j;
            dA[ABbegin + i + 7] = -B(2, 1);
        }
    }
    else if (delay_num_v == delay_num_w)
    {
        irowA[dimx] = 0;
        jcolA[dimx] = dimx;
        dA[dimx] = -B(0, 0);
        irowA[dimx+1] = 1;
        jcolA[dimx+1] = dimx;
        dA[dimx+1] = -B(1, 0);
        irowA[dimx+2] = 2;
        jcolA[dimx+2] = dimx+dimv;
        dA[dimx+2] = -B(2, 1);
        int ABidx = 8*(T-delay_num_v) - 8;
        int ABbegin = dimx+3;
        for (int i=0, j=1; i<ABidx; i+=8, j++)
        {
            getLinearModel(xbar[j+delay_num_v]);
            for (int k=0; k<3; k++)
            {
                b[3*j+k] = C[k];
                irowA[ABbegin + i + k] = 3*j + k;
                jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
                dA[ABbegin + i + k] = -A(k, k);
            }
            irowA[ABbegin + i + 3] = 3*j;
            jcolA[ABbegin + i + 3] = 3*j - 1;
            dA[ABbegin + i + 3] = -A(0, 2);

            irowA[ABbegin + i + 4] = 3*j + 1;
            jcolA[ABbegin + i + 4] = 3*j - 1;
            dA[ABbegin + i + 4] = -A(1, 2);
            
            irowA[ABbegin + i + 5] = 3*j;
            jcolA[ABbegin + i + 5] = dimx + j;
            dA[ABbegin + i + 5] = -B(0, 0);
            
            irowA[ABbegin + i + 6] = 3*j + 1;
            jcolA[ABbegin + i + 6] = dimx + j;
            dA[ABbegin + i + 6] = -B(1, 0);
            
            irowA[ABbegin + i + 7] = 3*j + 2;
            jcolA[ABbegin + i + 7] = dimx + dimv + j;
            dA[ABbegin + i + 7] = -B(2, 1);
        }
    }

    // iequality constraints
    const int mz  = dimw + dimv - 2;
    const int nnzC = 2 * mz;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    int k = 0;
    for (int i=0; i<dimv-1; i++, k+=2)
    {
        irowC[k] = i;
        jcolC[k] = dimx + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] + 1;
        dC[k+1] = 1.0;
    }
    for (int i=dimv-1; i<mz; i++, k+=2)
    {
        irowC[k] = i;
        jcolC[k] = dimx + i + 1;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] + 1;
        dC[k+1] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<dimv; i++)
    {
        lowerBound[i] = min_speed;
        upperBound[i] = max_speed;
        linearMatrix.insert(i, dimx+i) = 1;
    }
    for (int i=dimv; i<mx; i++)
    {
        lowerBound[i] = -max_omega;
        upperBound[i] = max_omega;
        linearMatrix.insert(i, i+dimx) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<dimv-1; i++)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
    }

    for (int i=dimv-1; i<mz; i++)
    {
        lowerBound[mx+my+i] = -max_comega;
        upperBound[mx+my+i] = max_comega;
    }
    // std::cout<<"hession="<<hessian<<std::endl;
    // std::cout<<"gradient="<<gradient.transpose()<<std::endl;
    // std::cout<<"linearMatrix="<<linearMatrix<<std::endl;
    // std::cout<<"lowerBound="<<lowerBound.transpose()<<std::endl;
    // std::cout<<"upperBound="<<upperBound.transpose()<<std::endl;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    // ROS_INFO("Solution: v0=%f     omega0=%f", QPSolution[dimx], QPSolution[dimx+1]);
    for (int i=0; i<delay_num_v; i++)
    {
        output(0, i) = output_buff[i][0];
    }
    for (int i=0; i<delay_num_w; i++)
    {
        output(1, i) = output_buff[i][1];
    }
    for (int i=0; i<dimv; i++)
    {
        output(0, i+delay_num_v) = QPSolution[dimx+i];
    }
    for (int i=0; i<dimw; i++)
    {
        output(1, i+delay_num_w) = QPSolution[dimx+dimv+i];
    }
}

void MPC::getCmd(void)
{
    int iter;
    ros::Time begin = ros::Time::now();
    for (iter=0; iter<max_iter; iter++)
    {
        predictMotion();
        last_output = output;
        if (model_type == DIFF)
            solveMPCDiff();
        else
            solveMPCDiff();
        double du = 0;
        for (int i=0; i<output.cols(); i++)
        {
            du = du + fabs(output(0, i) - last_output(0, i))+ fabs(output(1, i) - last_output(1, i));
        }
        // break;
        if (du <= du_th)
        {
            break;
        }
        if((ros::Time::now()-begin).toSec()>0.01){
            break;
        }
    }
    if (iter == max_iter)
    {
        ROS_WARN("MPC Iterative is max iter");
    }

    predictMotion(xopt);
    drawRefPath();
    drawPredictPath(xopt);
    
    cmd.speed = output(0, delay_num_v);
    cmd.omega = output(1, delay_num_w);

    if (max_delay_num>0)
    {
        output_buff.erase(output_buff.begin());
        output_buff.push_back(Eigen::Vector2d(output(0, delay_num_v),output(1, delay_num_w)));
    }
    
}