#pragma once

#include "mpc_controller/Polynome.h"
#include "mpc_controller/MincoTraj.h"
#include "mpc_controller/SinglePolyAC.h"
#include "mpc_controller/PolyTrajAC.h"
#include "mpc_controller/DPtrajContainer.h"
#include "utils/dptraj.hpp"
#include "utils/minco_traj.hpp"
#include <ros/ros.h>
#include <cmath>
#include <vector>

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;
    double v = 0;
    double w = 0;
};

class TrajAnalyzer{
private:
    mpc_utils::MincoTraj minco_anal;
    mpc_utils::Trajectory minco_traj;
    dptraj_mpc::UgvTrajectory ugv_traj;
    std::vector<mpc_utils::Trajectory> minco_trajs;
    std::vector<double> go_forwards;
    ros::Time start_time;
    double total_time;

public:

    bool at_goal = false;
    bool go_forward = true;

    TrajAnalyzer() {}

    void setTraj(mpc_controller::PolynomeConstPtr msg)
    {
      minco_trajs.clear();
      total_time = 0.0;
      go_forwards.clear();

      Eigen::MatrixXd posP(3, msg->pos_pts.size() - 2);
      Eigen::VectorXd T(msg->t_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
        posP(2, i - 1) = msg->pos_pts[i].z;
      }
      for (int i = 0; i < (int)msg->t_pts.size(); i++)
      {
        T(i) = msg->t_pts[i];
      }

      initS.setZero(3, 3);
      tailS.setZero(3, 3);
      initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
      initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
      initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
      tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
      tailS.col(1) = Eigen::Vector3d::Zero();
      tailS.col(2) = Eigen::Vector3d::Zero();
      minco_anal.reset(initS, msg->pos_pts.size() - 1);
      minco_anal.generate(posP, tailS, T);
      minco_traj = minco_anal.getTraj();

      minco_trajs.push_back(minco_traj);
      go_forwards.push_back(1.0);

      total_time = minco_traj.getTotalDuration();
      start_time = ros::Time::now();
    }
    void setTraj(mpc_controller::DPtrajContainerConstPtr msg)
    {
      total_time = 0.0;
      start_time = msg->start_time;

      dptraj_mpc::UgvTrajectory ugvTraj_tmp;
      for(int i = 0; i <msg->traj_container.size(); i++){
        std::vector<double> dts;
        std::vector<double> dss;
        std::vector<dptraj_mpc::CoefficientMat<2>> c2Mats;
        std::vector<dptraj_mpc::CoefficientMat<1>> c1Mats;
        for(int j = 0; j < msg->traj_container[i].trajs.size(); j++){
            dptraj_mpc::CoefficientMat<2> c2;
            dptraj_mpc::CoefficientMat<1> c1;
            for (int k=0; k<6; k++)
            {
              c2(0, k) = msg->traj_container[i].trajs[j].coef_x[k];
              c2(1, k) = msg->traj_container[i].trajs[j].coef_y[k];
              c1(0, k) = msg->traj_container[i].trajs[j].coef_s[k];
            }
            c2Mats.push_back(c2);
            c1Mats.push_back(c1);
            dts.push_back( msg->traj_container[i].trajs[j].dt);
            dss.push_back( msg->traj_container[i].trajs[j].ds);
        }
        dptraj_mpc::PolyTrajectory<2> r(dss, c2Mats);
        dptraj_mpc::PolyTrajectory<1> s(dts, c1Mats);
        dptraj_mpc::DpTrajectory dptraj;
        dptraj.posTraj = r;
        dptraj.tTraj = s;
        ugvTraj_tmp.Traj_container.push_back(dptraj);
        if(msg->reverse[i]>0){
          ugvTraj_tmp.etas.push_back(-1);
        }
        else{
          ugvTraj_tmp.etas.push_back(1);
        }
        // msg->traj_container[i].trajs
      }
      ugv_traj = ugvTraj_tmp;
      total_time = ugv_traj.getTotalDuration();
      ugv_traj.startTime = msg->start_time.toSec();
    }

    void setTraj(const dptraj_mpc::UgvTrajectory tj)
    {
      start_time = ros::Time::now();
      ugv_traj = tj;
      total_time = ugv_traj.getTotalDuration();
    }

    void setTraj(mpc_controller::MincoTrajConstPtr msgs)
    {
      minco_trajs.clear();
      total_time = 0.0;
      go_forwards.clear();

      for (size_t i=0; i<msgs->trajs.size(); i++)
      {
        mpc_controller::SingleMinco msg = msgs->trajs[i];
        Eigen::MatrixXd posP(3, msg.pos_pts.size() - 2);
        Eigen::VectorXd T(msg.t_pts.size());
        Eigen::MatrixXd initS, tailS;

        for (int i = 1; i < (int)msg.pos_pts.size() - 1; i++)
        {
          posP(0, i - 1) = msg.pos_pts[i].x;
          posP(1, i - 1) = msg.pos_pts[i].y;
          posP(2, i - 1) = 0.0;
        }
        for (int i = 0; i < (int)msg.t_pts.size(); i++)
        {
          T(i) = msg.t_pts[i];
        }

        initS.setZero(3, 3);
        tailS.setZero(3, 3);
        initS.col(0) = Eigen::Vector3d(msg.head_x.x, msg.head_y.x, 0.0);
        initS.col(1) = Eigen::Vector3d(msg.head_x.y, msg.head_y.y, 0.0);
        initS.col(2) = Eigen::Vector3d(msg.head_x.z, msg.head_y.z, 0.0);
        tailS.col(0) = Eigen::Vector3d(msg.tail_x.x, msg.tail_y.x, 0.0);
        tailS.col(1) = Eigen::Vector3d(msg.tail_x.y, msg.tail_y.y, 0.0);
        tailS.col(2) = Eigen::Vector3d(msg.tail_x.z, msg.tail_y.z, 0.0);
        minco_anal.reset(initS, msg.pos_pts.size() - 1);
        minco_anal.generate(posP, tailS, T);
        minco_traj = minco_anal.getTraj();

        minco_trajs.push_back(minco_traj);
        if (msg.go_forward)
          go_forwards.push_back(1.0);
        else
          go_forwards.push_back(-1.0);

        total_time += minco_traj.getTotalDuration();
      }

      start_time = ros::Time::now();
    }

    std::vector<TrajPoint> getTajWps(double dt)
    {
      std::vector<TrajPoint> P;
      TrajPoint tp;
      
      for (double t=0.0; t<=total_time; t+=dt)
      {
        Eigen::Vector3d po = minco_traj.getPos(t);
        tp.x = po[0];
        tp.y = po[1];
        tp.theta = po[2];
        P.push_back(tp);
      }
      return P;
    }

    void setTestTraj(double max_vel)
    {
      minco_trajs.clear();
      go_forwards.clear();

      Eigen::MatrixXd eight(10, 3);
      eight << 2.34191,      -0.382897,  0.127306, 
              3.09871,      0.936706,   1.72168,
              1.99125,      2.68782,    2.53673,
              0.394621,     3.877,      2.25751,
              -0.0799935,   5.86051,    1.13753,
              1.90338,      6.56037,    -0.143977,
              3.17197,      5.21122,    -1.66021,
              2.12699,      3.42104,    -2.48622,
              0.48492,      2.23846,    -2.31336,
              -0.00904252,  0.365258,   -1.55525;
      Eigen::MatrixXd posP = eight.transpose();
      Eigen::VectorXd T(11);
      T << 3.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.0;
      T *= 1.5 / max_vel;
      Eigen::MatrixXd initS, tailS;

      initS.setZero(3, 3);
      tailS.setZero(3, 3);
      initS.col(0) = Eigen::Vector3d(0, 0, -0.257661);
      tailS.col(0) = Eigen::Vector3d(0, 0, -1.54148);
      minco_anal.reset(initS, T.size());
      minco_anal.generate(posP, tailS, T);
      minco_traj = minco_anal.getTraj();
      total_time = minco_traj.getTotalDuration();

      minco_trajs.push_back(minco_traj);
      go_forwards.push_back(1.0);

      total_time = minco_traj.getTotalDuration();
      start_time = ros::Time::now();
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      int j = 0;
      double t_cur = time_now.toSec() - ugv_traj.startTime;
      if (t_cur > ugv_traj.getTotalDuration() + 1.0)
      {
        at_goal = true;
        return P;
      }
      else{
        at_goal = false;
      }
      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        
        Eigen::Vector2d pos = ugv_traj.getPos(t);
        double yaw = ugv_traj.getYaw(t);
        tp.x = pos[0];
        tp.y = pos[1];
        tp.theta = yaw;
        tp.v = ugv_traj.getVelItem(t);
        // tp.w = 
        P.push_back(tp);
      }

      return P;



      /*
      double t_cur = (time_now - start_time).toSec();
      int j=0;
      if (t_cur > total_time + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        double temp = t;
        bool gedit = false;
        for (size_t i=0 ;i<minco_trajs.size(); i++)
        {
          if (temp-minco_trajs[i].getTotalDuration() < 0)
          {
            Eigen::Vector3d po = minco_trajs[i].getPos(temp);
            Eigen::Vector3d vo = minco_trajs[i].getVel(temp);
            // Eigen::Vector3d ao = minco_trajs[i].getAcc(temp);

            tp.x = po(0);
            tp.y = po(1);
            tp.theta = atan2(go_forwards[i]*vo(1), go_forwards[i]*vo(0));

            // double vv = std::sqrt(vo(1)*vo(1)+vo(0)*vo(0));
            // double aa = std::sqrt(ao(1)*ao(1)+ao(0)*ao(0));
            // if (vv > 1.5 || aa > 0.8)
            // {
            //   std::cout<<"shit vel: "<<vv<<std::endl;
            //   std::cout<<"shit acc: "<<aa<<std::endl;
            // }
            at_goal = false;
            P.push_back(tp);
            gedit = true;
            if (j==0)
              go_forward = (go_forwards[i]>0.0);
            break;
          }
          else
          {
            temp -= minco_trajs[i].getTotalDuration();
          }
        }
        if (gedit == false)
        {
          mpc_utils::Trajectory back_traj = minco_trajs.back();
          Eigen::Vector3d po = back_traj.getPos(back_traj.getTotalDuration());
          Eigen::Vector3d vo = back_traj.getVel(back_traj.getTotalDuration());
          
          tp.x = po(0);
          tp.y = po(1);
          tp.theta = atan2(go_forwards.back()*vo(1), go_forwards.back()*vo(0));
          P.push_back(tp);
          if (j==0)
              go_forward = (go_forwards.back()>0.0);
        }
      }

      return P;*/


    }
   
    ~TrajAnalyzer() {}
};