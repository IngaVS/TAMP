/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/* This is an old version of FIRI for temporary usage here. */


#ifndef FIRI_2D_HPP_GCOPTER
#define FIRI_2D_HPP_GCOPTER

#include "mvie2d.hpp"
#include "sdqp.hpp"
#include <Eigen/Eigen>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <vector>
#include <omp.h>

namespace firi2d
{
    inline bool firi(const Eigen::MatrixX3d &bd,
                     const Eigen::Matrix2Xd &pc,
                     const Eigen::Vector2d &a,
                     const Eigen::Vector2d &b,
                     Eigen::MatrixX3d &hPoly,
                     int &iter_num,
                     const int iterations = 6,
                     const double epsilon = 1.0e-6)
    {
        const Eigen::Vector3d ah(a(0), a(1), 1.0);
        const Eigen::Vector3d bh(b(0), b(1), 1.0);

        if ((bd * ah).maxCoeff() > 0.0 ||
            (bd * bh).maxCoeff() > 0.0)
        {
            return false;
        }

        const int M = bd.rows();
        const int N = pc.cols();

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d p = 0.5 * (a + b);
        Eigen::Vector2d r = Eigen::Vector2d::Ones() * 0.001;
        double last_volume = r.prod();

        Eigen::MatrixX3d forwardH(M + N, 3);
        int nH = 0;
        iter_num = 0;

        for (int loop = 0; loop < iterations; ++loop)
        {
            iter_num++;

            // TicToc t_RESTRICTIVE_INFLATION;

            const Eigen::Matrix2d forward = r.cwiseInverse().asDiagonal() * R.transpose();
            const Eigen::Matrix2d backward = R * r.asDiagonal();
            const Eigen::MatrixX2d forwardB = bd.leftCols<2>() * backward;
            const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<2>() * p;
            const Eigen::Vector2d neg_forwardp = forward * (-p);
            const Eigen::Matrix2Xd forwardPC = (forward * pc).colwise() + neg_forwardp;
            const Eigen::Vector2d fwd_a = forward * (a - p);
            const Eigen::Vector2d fwd_b = forward * (b - p);

            Eigen::VectorXd distDs = forwardD.array().square();
            distDs = distDs.cwiseQuotient(forwardB.rowwise().squaredNorm());
            // const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());

            Eigen::MatrixX3d tangents(N, 3);
            Eigen::VectorXd distRs(N);

            bool path_in_ball = false;
            if(fwd_a.norm() < 1.0 && fwd_b.norm() < 1.0)
                path_in_ball = true;

            distRs = forwardPC.colwise().squaredNorm();
            tangents.col(2) = -distRs;
            tangents.leftCols(2) = forwardPC.transpose();

            if( !path_in_ball )
            {
                for (int i = 0; i < N; i++)
                {
                    if (tangents.block<1, 2>(i, 0).dot(fwd_a) + tangents(i, 2) > epsilon)
                    {
                        const Eigen::Vector2d delta = forwardPC.col(i) - fwd_a;
                        tangents.block<1, 2>(i, 0) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                        distRs(i) = tangents.block<1, 2>(i, 0).squaredNorm();
                        tangents(i, 2) = -distRs(i);
                        // tangents.block<1, 2>(i, 0) /= distRs(i);
                    }
                    if (tangents.block<1, 2>(i, 0).dot(fwd_b) + tangents(i, 2) > epsilon)
                    {
                        const Eigen::Vector2d delta = forwardPC.col(i) - fwd_b;
                        tangents.block<1, 2>(i, 0) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                        distRs(i) = tangents.block<1, 2>(i, 0).squaredNorm();
                        tangents(i, 2) = -distRs(i);
                        // tangents.block<1, 2>(i, 0) /= distRs(i);
                    }
                }
            }

            // std::cout << "RESTRICTIVE_INFLATION: tangents: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;


            // t_RESTRICTIVE_INFLATION.tic();

            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            nH = 0;

            bool completed = false;
            int bdMinId = 0, pcMinId = 0;
            double minSqrD = distDs.minCoeff(&bdMinId);
            double minSqrR = INFINITY;
            Eigen::Vector2d norm_nH;
            Eigen::MatrixXd norm_nH_T(1, 2);
            // Eigen::MatrixXd wxx(1, N);
            double b_nH;
            if (distRs.size() != 0)
            {
                minSqrR = distRs.minCoeff(&pcMinId);
            }
            int i = 0;
            for (; !completed && i < (M + N); ++i)
            {
                if (minSqrD < minSqrR)
                {
                    forwardH.block<1, 2>(nH, 0) = forwardB.row(bdMinId);
                    forwardH(nH, 2) = forwardD(bdMinId);
                    bdFlags(bdMinId) = 0;
                }
                else
                {
                    forwardH.row(nH) = tangents.row(pcMinId);
                    pcFlags(pcMinId) = 0;
                }

                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j)
                {
                    if (bdFlags(j))
                    {
                        completed = false;
                        if (minSqrD > distDs(j))
                        {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                norm_nH = forwardH.block<1, 2>(nH, 0);
                norm_nH_T = norm_nH.transpose();
                b_nH = forwardH(nH, 2);
                double neg_b_epsilon = -epsilon - b_nH;
                
                for (int j = 0; j < N; ++j)
                {
                    if (pcFlags(j))
                    {
                        if (norm_nH.dot(forwardPC.col(j)) > neg_b_epsilon)
                        {
                            pcFlags(j) = 0;
                        }
                        else
                        {
                            completed = false;
                            if (minSqrR > distRs(j))
                            {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                
                ++nH;
            }
            //  // wxx << M+N << std::endl;
            //  // wxx << i << std::endl;

            // std::cout << "RESTRICTIVE_INFLATION: sort: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;

            hPoly.resize(nH, 3);
            for (int i = 0; i < nH; ++i)
            {
                hPoly.block<1, 2>(i, 0) = forwardH.block<1, 2>(i, 0) * forward;
                hPoly(i, 2) = forwardH(i, 2) - hPoly.block<1, 2>(i, 0).dot(p);
            }
            //  // wxx << "RESTRICTIVE_INFLATION: total: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;


            if (loop == iterations - 1)
            {
                break;
            }

            //  // wxx << "hPoly: " << hPoly << std::endl;

            // TicToc t_MVIE;
            if(mvie2d::mvie2d(hPoly.leftCols<2>(), -hPoly.col(2), R, p, r))
            {
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
            }
            // std::cout << "MVIE: " << t_MVIE.toc() << std::endl;

            double current_volume = r.prod();
            //  // wxx << "r: " << r.transpose() << std::endl;
            if( (current_volume - last_volume) / last_volume < 1e-2 )
            {
                 // wxx << "iteration time: " << loop << std::endl;
                break;
            }
            last_volume = current_volume;
        }

        return true;
    }

    inline bool firi(const Eigen::MatrixX3d &bd,
                     const Eigen::Matrix2Xd &pc,
                     const Eigen::Matrix2Xd &robot,
                     Eigen::MatrixX3d &hPoly,
                     int &iter_num,
                     const int iterations = 6,
                     const double epsilon = 1.0e-6)
    {

        Eigen::Vector2d p{0, 0};
        const int robot_N = robot.cols();
        for( int i = 0; i < robot_N; i++ )
        {
            p += robot.col(i);
            Eigen::Vector3d roboth{ robot.col(i)[0], robot.col(i)[1], 1.0 };
            if( (bd * roboth).maxCoeff() > 0.0 )
                return false;
        }
        p = p / (robot_N * 1.0);

        const int M = bd.rows();
        const int N = pc.cols();

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d r = Eigen::Vector2d::Ones() * 0.001;
        double last_volume = r.prod();

        Eigen::MatrixX3d forwardH(M + N, 3);
        int nH = 0;
        iter_num = 0;

        for (int loop = 0; loop < iterations; ++loop)
        {
            iter_num++;

            // TicToc t_RESTRICTIVE_INFLATION;

            const Eigen::Matrix2d forward = r.cwiseInverse().asDiagonal() * R.transpose();
            const Eigen::Matrix2d backward = R * r.asDiagonal();
            const Eigen::MatrixX2d forwardB = bd.leftCols<2>() * backward;
            const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<2>() * p;
            const Eigen::Vector2d neg_forwardp = forward * (-p);
            const Eigen::Matrix2Xd forwardPC = (forward * pc).colwise() + neg_forwardp;
            const Eigen::Matrix2Xd forwardRobot = forward * (robot.colwise() - p);

            Eigen::VectorXd distDs = forwardD.array().square();
            distDs = distDs.cwiseQuotient(forwardB.rowwise().squaredNorm());
            // const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());

            Eigen::MatrixX3d tangents(N, 3);
            Eigen::VectorXd distRs(N);

            bool path_in_ball = false;
            if(forwardRobot.colwise().norm().maxCoeff() < 1.0)
                path_in_ball = true;

            if( path_in_ball )
            {
                distRs = forwardPC.colwise().squaredNorm();
                tangents.col(2) = -distRs;
                tangents.leftCols(2) = forwardPC.transpose();
            }
            else
            {
                Eigen::Matrix<double, 3, -1> AbT(3, robot_N + 1);
                for( int i = 1; i < robot_N+1; i++ )
                {
                    AbT.col(i).topRows(2) = forwardRobot.col(i-1);
                    AbT.col(i)[2] = -1;
                }
                for (int i = 0; i < N; i++)
                {
                    AbT.col(0).topRows(2) = -forwardPC.col(i);
                    AbT.col(0)[2] = 1;
                    Eigen::Vector2d a, b;
                    double minobj = sdqp::sdqp_min_norm<2>(AbT, b);
                    a = b / (b.dot(b));
                    double a_norm2 = a.squaredNorm();
                    tangents.block<1, 2>(i, 0) = a.transpose();
                    tangents(i, 2) = -a_norm2;
                    distRs(i) = a_norm2;
                }
            }

             // wxx << "RESTRICTIVE_INFLATION: tangents: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;


            // t_RESTRICTIVE_INFLATION.tic();

            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            nH = 0;

            bool completed = false;
            int bdMinId = 0, pcMinId = 0;
            double minSqrD = distDs.minCoeff(&bdMinId);
            double minSqrR = INFINITY;
            Eigen::Vector2d norm_nH;
            Eigen::MatrixXd norm_nH_T(1, 2);
            Eigen::MatrixXd wxx(1, N);
            double b_nH;
            if (distRs.size() != 0)
            {
                minSqrR = distRs.minCoeff(&pcMinId);
            }
            int i = 0;
            for (; !completed && i < (M + N); ++i)
            {
                if (minSqrD < minSqrR)
                {
                    forwardH.block<1, 2>(nH, 0) = forwardB.row(bdMinId);
                    forwardH(nH, 2) = forwardD(bdMinId);
                    bdFlags(bdMinId) = 0;
                }
                else
                {
                    forwardH.row(nH) = tangents.row(pcMinId);
                    pcFlags(pcMinId) = 0;
                }

                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j)
                {
                    if (bdFlags(j))
                    {
                        completed = false;
                        if (minSqrD > distDs(j))
                        {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                norm_nH = forwardH.block<1, 2>(nH, 0);
                norm_nH_T = norm_nH.transpose();
                b_nH = forwardH(nH, 2);
                double neg_b_epsilon = -epsilon - b_nH;
                
                for (int j = 0; j < N; ++j)
                {
                    if (pcFlags(j))
                    {
                        if (norm_nH.dot(forwardPC.col(j)) > neg_b_epsilon)
                        {
                            pcFlags(j) = 0;
                        }
                        else
                        {
                            completed = false;
                            if (minSqrR > distRs(j))
                            {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                
                ++nH;
            }
            //  // wxx << M+N << std::endl;
            //  // wxx << i << std::endl;

             // wxx << "RESTRICTIVE_INFLATION: sort: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;

            hPoly.resize(nH, 3);
            for (int i = 0; i < nH; ++i)
            {
                hPoly.block<1, 2>(i, 0) = forwardH.block<1, 2>(i, 0) * forward;
                hPoly(i, 2) = forwardH(i, 2) - hPoly.block<1, 2>(i, 0).dot(p);
            }
            //  // wxx << "RESTRICTIVE_INFLATION: total: " << t_RESTRICTIVE_INFLATION.toc() << std::endl;


            if (loop == iterations - 1)
            {
                break;
            }

            //  // wxx << "hPoly: " << hPoly << std::endl;

            // TicToc t_MVIE;
            if(mvie2d::mvie2d(hPoly.leftCols<2>(), -hPoly.col(2), R, p, r))
            {
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
                 // wxx << "\033[1;34m[MVIE] ERROR!!!" << "\033[0m" << std::endl;
            }
             // wxx << "MVIE: " << t_MVIE.toc() << std::endl;

            double current_volume = r.prod();
            //  // wxx << "r: " << r.transpose() << std::endl;
            if( (current_volume - last_volume) / last_volume < 2e-2 )
            {
                 // wxx << "iteration time: " << loop << std::endl;
                break;
            }
            last_volume = current_volume;
        }

        return true;
    }
}

#endif
