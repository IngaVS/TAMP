/*
    MIT License

    Copyright (c) 2022 Zhepei Wang (wangzhepei@live.com)

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

#ifndef SDQP_HPP
#define SDQP_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <random>
#include "tools/tic_toc.hpp"

namespace sdqp
{
    constexpr double eps = 1.0e-12;
    double time;
    int norm1_count = 0;

    double time_norm1 = 0;
    double time_norm1_pre = 0;
    double time_out = 0;

    typedef std::uniform_int_distribution<int> rand_int;
    typedef rand_int::param_type rand_range;
    static std::mt19937_64 gen;
    static rand_int rdi(0, 1);

    enum
    {
        MINIMUM = 0,
        INFEASIBLE,
    };

    template <int d>
    inline void set_zero(double *x)
    {
        for (int i = 0; i < d; ++i)
        {
            x[i] = 0.0;
        }
        return;
    }

    template <int d>
    inline double dot(const double *x,
                      const double *y)
    {
        double s = 0.0;
        for (int i = 0; i < d; ++i)
        {
            s += x[i] * y[i];
        }
        return s;
    }

    template <int d>
    inline double sqr_norm(const double *x)
    {
        double s = 0.0;
        for (int i = 0; i < d; ++i)
        {
            s += x[i] * x[i];
        }
        return s;
    }

    template <int d>
    inline void mul(const double *x,
                    const double s,
                    double *y)
    {
        for (int i = 0; i < d; ++i)
        {
            y[i] = x[i] * s;
        }
        return;
    }

    template <int d>
    inline int max_abs(const double *x)
    {
        int id = 0;
        double mag = std::fabs(x[0]);
        for (int i = 1; i < d; ++i)
        {
            const double s = std::fabs(x[i]);
            if (s > mag)
            {
                id = i;
                mag = s;
            }
        }
        return id;
    }

    template <int d>
    inline void cpy(const double *x,
                    double *y)
    {
        for (int i = 0; i < d; ++i)
        {
            y[i] = x[i];
        }
        return;
    }

    inline int move_to_front(const int i,
                             int *next,
                             int *prev)
    {
        if (i == 0 || i == next[0])
        {
            return i;
        }
        const int previ = prev[i];
        next[prev[i]] = next[i];
        prev[next[i]] = prev[i];
        next[i] = next[0];
        prev[i] = 0;
        prev[next[i]] = i;
        next[0] = i;
        return previ;
    }

    template <int d>
    inline int min_norm(const double *halves,
                        const int n,
                        const int m,
                        double *opt,
                        double *work)
    {
        int status = MINIMUM;
        set_zero<d>(opt);
        if (m <= 0)
        {
            return status;
        }

        double *reflx = work;
        double *new_opt = reflx + d;
        double *new_halves = new_opt + (d - 1);
        double *new_work = new_halves + n * d;
        TicToc timer_out;
        // TicToc timer_out_out;
        double eps2 = (d + 1) * eps * eps;

        for (int i = 0; i < m; i++)
        {
            const double *plane_i = halves + (d + 1) * i;

            if (dot<d>(opt, plane_i) + plane_i[d] > (d + 1) * eps)
            {
                const double s = sqr_norm<d>(plane_i);
                if (s < eps2)
                {
                    return INFEASIBLE;
                }

                mul<d>(plane_i, -plane_i[d] / s, opt);
                
                if (i == 0)
                {
                    continue;
                }


                // TicToc timer;
                // timer.tic();
                // stable Householder reflection with pivoting
                const int id = max_abs<d>(opt);
                const double xnorm = std::sqrt(sqr_norm<d>(opt));
                cpy<d>(opt, reflx);
                reflx[id] += opt[id] < 0.0 ? -xnorm : xnorm;
                const double h = -2.0 / sqr_norm<d>(reflx);

                // std::cout << "d: " << d << ", i: " << i << std::endl;
                for (int j = 0; j < i; j++)
                {
                    double *new_plane = new_halves + d * j;
                    const double *old_plane = halves + (d + 1) * j;
                    const double coeff = h * dot<d>(old_plane, reflx);
                    for (int k = 0; k < d; ++k)
                    {
                        const int l = k < id ? k : k - 1;
                        new_plane[l] = k != id ? old_plane[k] + reflx[k] * coeff : new_plane[l];
                    }
                    new_plane[d - 1] = dot<d>(opt, old_plane) + old_plane[d];
                }
                // time_norm1_pre += timer.toc();

                status = min_norm<d - 1>(new_halves, n, i, new_opt, new_work);

                if (status == INFEASIBLE)
                {
                    return INFEASIBLE;
                }

                double coeff = 0.0;
                for (int j = 0; j < d; ++j)
                {
                    const int k = j < id ? j : j - 1;
                    coeff += j != id ? reflx[j] * new_opt[k] : 0.0;
                }
                coeff *= h;
                for (int j = 0; j < d; ++j)
                {
                    const int k = j < id ? j : j - 1;
                    opt[j] += j != id ? new_opt[k] + reflx[j] * coeff : reflx[j] * coeff;
                }

                // i = move_to_front(i, next, prev);
                // time_out += timer_out.toc();
            }
            // else
                // time_out += timer_out.toc();
        }
        // double time_out_out = timer_out_out.toc();
        // std::cout << "time out out: " << time_out_out << std::endl;

        return status;
    }

    template <>
    inline int min_norm<1>(const double *halves,
                           const int n,
                           const int m,
                           double *opt,
                           double *work)
    {
        opt[0] = 0.0;
        bool l = false;
        bool r = false;

        TicToc timer;
        timer.tic();

        for (int i = 0; i < m; i++)
        {
            const double a = halves[2 * i];
            const double b = halves[2 * i + 1];
            if (a * opt[0] + b > 2.0 * eps)
            {
                if (std::fabs(a) < 2.0 * eps)
                {
                    return INFEASIBLE;
                }

                l = l || a < 0.0;
                r = r || a > 0.0;

                if (l && r)
                {
                    return INFEASIBLE;
                }

                opt[0] = -b / a;
            }
        }
        norm1_count++;
        // std::cout << norm1_count << std::endl;
        time_norm1 += timer.toc();

        return MINIMUM;
    }


    /**
     * minimize     0.5 x' Q x + c' x
     * subject to       A x <= b
     * Q must be positive definite
     **/
    template <int d>
    inline double sdqp_min_norm(const Eigen::Matrix<double, d+1, -1> &halves,
                                Eigen::Matrix<double, d, 1> &x)
    {
        x.setZero();
        const int n = halves.cols();
        if (n < 1)
        {
            return 0.0;
        }

        Eigen::VectorXd work((n + 2) * (d + 2) * (d - 1) / 2 + 1 - d);
        int size = n * (d+1);

        const int status = min_norm<d>(halves.data(), n, n,
                                       x.data(), work.data());

        double minimum = INFINITY;
        if (status != INFEASIBLE)
        {
            minimum = x.norm();
        }
 
        // if (!std::isinf(minimum))
        // {
        //     llt.matrixU().template solveInPlace<Eigen::OnTheLeft>(x);
        //     x -= Eigen::Vector2d{1, 1};
        //     minimum = 0.5 * (Q * x).dot(x) + c.dot(x);
        // }

        return minimum;
    }

} // namespace sdqp

#endif
