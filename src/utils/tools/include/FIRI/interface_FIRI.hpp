#ifndef INTERFACE_FIRI_HPP
#define INTERFACE_FIRI_HPP

#include <Eigen/Eigen>
#include <memory>
#include <decomp_basis/data_type.h>
#include "firi2d.hpp"

class Interface_FIRI 
{
public:
    Interface_FIRI() = default;
    ~Interface_FIRI(){}

    void AB2hPoly(Eigen::MatrixX3d &hPoly_in, Eigen::MatrixXd &hPoly)
    {
      int size = hPoly_in.rows();
      hPoly.resize(4, size);
      for( int i = 0; i < size; i ++ )
      {
        Eigen::Vector2d norm = hPoly_in.row(i).head(2).transpose();
        double norm_length = norm.norm(); 
        norm.normalize();
        hPoly.col(i).topRows<2>() = norm;

        Eigen::Vector2d point{0, 0};
        int max_norm_index;
        double max_norm_e = norm.cwiseAbs().maxCoeff(&max_norm_index);
        for( int j = 0; j < 2; j++ )
        {
          if( j == max_norm_index )
          {
            point[j] = -hPoly_in.row(i)[2] / (norm[j] * norm_length);
          }
        }

        hPoly.col(i).bottomRows<2>() = point;
      }
      return;
    }

    void reset()
    {
      have_seed_ = false;
      iter_ = 0;
    }

    void setMap(Eigen::Vector2d highCorner, Eigen::Vector2d lowCorner)
    {
      highCorner_ = highCorner;
      lowCorner_  = lowCorner;
    }

    void setSeedLine(double seed_line)
    {
      seed_line_ = seed_line;
    }

    void setSeedPoint( Eigen::Vector2d seed_point )
    {
      have_seed_ = true;
      seed_point_ = seed_point;
      seed_point_a_ = seed_point + Eigen::Vector2d{seed_line_, 0.0};
      seed_point_b_ = seed_point - Eigen::Vector2d{seed_line_, 0.0};
      return;
    }

    void addObstacle( std::vector<Eigen::Vector2d> obstacle )
    {
      Eigen::Map<const Eigen::Matrix<double, 2, -1, Eigen::ColMajor>> pc(obstacle[0].data(), 2, obstacle.size());
      obs_pc_ = pc;
      return;
    }

    void addBound( double bound )
    {
      if( !have_seed_ )
      {
        std::cout << "\033[1;31m[interface FIRI]: do not have seed point before add bound" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m[interface FIRI]: do not have seed point before add bound" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m[interface FIRI]: do not have seed point before add bound" << "\033[0m" << std::endl;
      }

      Eigen::Matrix<double, 4, 3> bd = Eigen::Matrix<double, 4, 3>::Zero();
      bd(0, 0) = 1.0;
      bd(1, 0) = -1.0;
      bd(2, 1) = 1.0;
      bd(3, 1) = -1.0;
      double range = bound;
      Eigen::Vector2d highCorner = highCorner_;
      Eigen::Vector2d lowCorner = lowCorner_;
      Eigen::Vector2d a, b;
      a = seed_point_a_;
      b = seed_point_b_;
      bd(0, 2) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
      bd(1, 2) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
      bd(2, 2) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
      bd(3, 2) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
      bd_ = bd;

      return;
    }

    void run( Eigen::MatrixXd &hPoly, double &run_time )
    {
      TicToc timer;
      Eigen::MatrixX3d hp;
      firi2d::firi(bd_, obs_pc_, seed_point_a_, seed_point_b_, hp, iter_);

      run_time = timer.toc();

      hPoly.resize(hp.rows(), 3);
      hPoly = hp;
      return;
    }

    int getIter(){ return iter_; }
    void quickRun( Eigen::Vector2d seed_point, std::vector<Eigen::Vector2d> obstacle, double bound, Eigen::MatrixXd &hPoly, double &run_time )
    {
      reset();
      setSeedPoint(seed_point);
      addObstacle(obstacle);
      addBound(bound);
      run(hPoly, run_time);
      return;
    }

protected:
    int iter_ = 0;
    double seed_line_ = 0.02;
    Eigen::MatrixX3d bd_;
    Eigen::Matrix2Xd obs_pc_;
    bool have_seed_ = false;
    Eigen::Vector2d seed_point_, seed_point_a_, seed_point_b_;
    Eigen::Vector2d highCorner_, lowCorner_;
};

class Interface_FIRI_Robot : public Interface_FIRI
{
public:
    Interface_FIRI_Robot() = default;
    ~Interface_FIRI_Robot(){}

    void setRobot( const Eigen::Matrix2Xd &robot_pc ) 
    {
      robot_pc_ = robot_pc;
    }

    void run( Eigen::MatrixXd &hPoly, double &run_time )
    {
      TicToc timer;
      Eigen::MatrixX3d hp;
      firi2d::firi(bd_, obs_pc_, robot_pc_, hp, iter_);
      run_time = timer.toc();

      AB2hPoly(hp, hPoly);
      return;
    }

    int getIter(){ return iter_; }
    void quickRun( Eigen::Vector2d seed_point, const Eigen::Matrix2Xd &robot_pc, std::vector<Eigen::Vector2d> obstacle, double bound, Eigen::MatrixXd &hPoly, double &run_time )
    {
      reset();
      setSeedPoint(seed_point);
      setRobot(robot_pc);
      addObstacle(obstacle);
      addBound(bound);
      run(hPoly, run_time);
      return;
    }

private:
    Eigen::Matrix2Xd robot_pc_;
};

#endif  // INTERFACE_FIRI_HPP