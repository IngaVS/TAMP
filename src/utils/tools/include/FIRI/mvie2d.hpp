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

#ifndef MVIE2D_HPP
#define MVIE2D_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <random>

namespace mvie2d
{

    constexpr double eps = 1.0e-12;

    inline bool h2v_3(const double *h0,
                      const double *h1,
                      const double *h2,
                      double *v0,
                      double *v1,
                      double *v2)
    {
        const double det0 = h0[0] * h1[1] - h0[1] * h1[0];
        if (std::fabs(det0) < eps * eps)
        {
            return true;
        }
        v0[0] = (h0[1] * h1[2] - h0[2] * h1[1]) / det0;
        v0[1] = (h0[2] * h1[0] - h0[0] * h1[2]) / det0;

        const double det1 = h1[0] * h2[1] - h1[1] * h2[0];
        if (std::fabs(det1) < eps * eps)
        {
            return true;
        }
        v1[0] = (h1[1] * h2[2] - h1[2] * h2[1]) / det1;
        v1[1] = (h1[2] * h2[0] - h1[0] * h2[2]) / det1;

        const double det2 = h2[0] * h0[1] - h2[1] * h0[0];
        if (std::fabs(det2) < eps * eps)
        {
            return true;
        }
        v2[0] = (h2[1] * h0[2] - h2[2] * h0[1]) / det2;
        v2[1] = (h2[2] * h0[0] - h2[0] * h0[2]) / det2;

        return false;
    }

    inline bool h2v_ccw_4(const double *h0,
                          const double *h1,
                          const double *h2,
                          const double *h3,
                          double *v0,
                          double *v1,
                          double *v2,
                          double *v3)
    {
        // TODO 这个排序是不是可以增量式的
        const double *h;
        if (h0[0] >= 0.0 && h1[0] < 0.0 ? true : (h0[0] < 0.0 && h1[0] >= 0.0 ? false : h0[0] * h1[1] - h0[1] * h1[0] < 0.0))
        {
            h = h0;
            h0 = h1;
            h1 = h;
        }
        if (h2[0] >= 0.0 && h3[0] < 0.0 ? true : (h2[0] < 0.0 && h3[0] >= 0.0 ? false : h2[0] * h3[1] - h2[1] * h3[0] < 0.0))
        {
            h = h2;
            h2 = h3;
            h3 = h;
        }
        if (h0[0] >= 0.0 && h2[0] < 0.0 ? true : (h0[0] < 0.0 && h2[0] >= 0.0 ? false : h0[0] * h2[1] - h0[1] * h2[0] < 0.0))
        {
            h = h0;
            h0 = h2;
            h2 = h;
        }
        if (h1[0] >= 0.0 && h3[0] < 0.0 ? true : (h1[0] < 0.0 && h3[0] >= 0.0 ? false : h1[0] * h3[1] - h1[1] * h3[0] < 0.0))
        {
            h = h1;
            h1 = h3;
            h3 = h;
        }
        if (h1[0] >= 0.0 && h2[0] < 0.0 ? true : (h1[0] < 0.0 && h2[0] >= 0.0 ? false : h1[0] * h2[1] - h1[1] * h2[0] < 0.0))
        {
            h = h1;
            h1 = h2;
            h2 = h;
        }

        const double det0 = h0[0] * h1[1] - h0[1] * h1[0];
        if (std::fabs(det0) < eps * eps)
        {
            return true;
        }
        v0[0] = (h0[1] * h1[2] - h0[2] * h1[1]) / det0;
        v0[1] = (h0[2] * h1[0] - h0[0] * h1[2]) / det0;

        const double det1 = h1[0] * h2[1] - h1[1] * h2[0];
        if (std::fabs(det1) < eps * eps)
        {
            return true;
        }
        v1[0] = (h1[1] * h2[2] - h1[2] * h2[1]) / det1;
        v1[1] = (h1[2] * h2[0] - h1[0] * h2[2]) / det1;

        const double det2 = h2[0] * h3[1] - h2[1] * h3[0];
        if (std::fabs(det2) < eps * eps)
        {
            return true;
        }
        v2[0] = (h2[1] * h3[2] - h2[2] * h3[1]) / det2;
        v2[1] = (h2[2] * h3[0] - h2[0] * h3[2]) / det2;

        const double det3 = h3[0] * h0[1] - h3[1] * h0[0];
        if (std::fabs(det3) < eps * eps)
        {
            return true;
        }
        v3[0] = (h3[1] * h0[2] - h3[2] * h0[1]) / det3;
        v3[1] = (h3[2] * h0[0] - h3[0] * h0[2]) / det3;

        return false;
    }

    inline bool h2v_ccw_5(const double *h0,
                          const double *h1,
                          const double *h2,
                          const double *h3,
                          const double *h4,
                          double *v0,
                          double *v1,
                          double *v2,
                          double *v3,
                          double *v4)
    {
        const double *h;
        if (h0[0] >= 0.0 && h4[0] < 0.0 ? true : (h0[0] < 0.0 && h4[0] >= 0.0 ? false : h0[0] * h4[1] - h0[1] * h4[0] < 0.0))
        {
            h = h0;
            h0 = h4;
            h4 = h;
        }
        if (h0[0] >= 0.0 && h2[0] < 0.0 ? true : (h0[0] < 0.0 && h2[0] >= 0.0 ? false : h0[0] * h2[1] - h0[1] * h2[0] < 0.0))
        {
            h = h0;
            h0 = h2;
            h2 = h;
        }
        if (h1[0] >= 0.0 && h3[0] < 0.0 ? true : (h1[0] < 0.0 && h3[0] >= 0.0 ? false : h1[0] * h3[1] - h1[1] * h3[0] < 0.0))
        {
            h = h1;
            h1 = h3;
            h3 = h;
        }
        if (h2[0] >= 0.0 && h4[0] < 0.0 ? true : (h2[0] < 0.0 && h4[0] >= 0.0 ? false : h2[0] * h4[1] - h2[1] * h4[0] < 0.0))
        {
            h = h2;
            h2 = h4;
            h4 = h;
        }
        if (h0[0] >= 0.0 && h1[0] < 0.0 ? true : (h0[0] < 0.0 && h1[0] >= 0.0 ? false : h0[0] * h1[1] - h0[1] * h1[0] < 0.0))
        {
            h = h0;
            h0 = h1;
            h1 = h;
        }
        if (h2[0] >= 0.0 && h3[0] < 0.0 ? true : (h2[0] < 0.0 && h3[0] >= 0.0 ? false : h2[0] * h3[1] - h2[1] * h3[0] < 0.0))
        {
            h = h2;
            h2 = h3;
            h3 = h;
        }
        if (h1[0] >= 0.0 && h4[0] < 0.0 ? true : (h1[0] < 0.0 && h4[0] >= 0.0 ? false : h1[0] * h4[1] - h1[1] * h4[0] < 0.0))
        {
            h = h1;
            h1 = h4;
            h4 = h;
        }
        if (h1[0] >= 0.0 && h2[0] < 0.0 ? true : (h1[0] < 0.0 && h2[0] >= 0.0 ? false : h1[0] * h2[1] - h1[1] * h2[0] < 0.0))
        {
            h = h1;
            h1 = h2;
            h2 = h;
        }
        if (h3[0] >= 0.0 && h4[0] < 0.0 ? true : (h3[0] < 0.0 && h4[0] >= 0.0 ? false : h3[0] * h4[1] - h3[1] * h4[0] < 0.0))
        {
            h = h3;
            h3 = h4;
            h4 = h;
        }

        const double det0 = h0[0] * h1[1] - h0[1] * h1[0];
        if (std::fabs(det0) < eps * eps)
        {
            return true;
        }
        v0[0] = (h0[1] * h1[2] - h0[2] * h1[1]) / det0;
        v0[1] = (h0[2] * h1[0] - h0[0] * h1[2]) / det0;

        const double det1 = h1[0] * h2[1] - h1[1] * h2[0];
        if (std::fabs(det1) < eps * eps)
        {
            return true;
        }
        v1[0] = (h1[1] * h2[2] - h1[2] * h2[1]) / det1;
        v1[1] = (h1[2] * h2[0] - h1[0] * h2[2]) / det1;

        const double det2 = h2[0] * h3[1] - h2[1] * h3[0];
        if (std::fabs(det2) < eps * eps)
        {
            return true;
        }
        v2[0] = (h2[1] * h3[2] - h2[2] * h3[1]) / det2;
        v2[1] = (h2[2] * h3[0] - h2[0] * h3[2]) / det2;

        const double det3 = h3[0] * h4[1] - h3[1] * h4[0];
        if (std::fabs(det3) < eps * eps)
        {
            return true;
        }
        v3[0] = (h3[1] * h4[2] - h3[2] * h4[1]) / det3;
        v3[1] = (h3[2] * h4[0] - h3[0] * h4[2]) / det3;

        const double det4 = h4[0] * h0[1] - h4[1] * h0[0];
        if (std::fabs(det4) < eps * eps)
        {
            return true;
        }
        v4[0] = (h4[1] * h0[2] - h4[2] * h0[1]) / det4;
        v4[1] = (h4[2] * h0[0] - h4[0] * h0[2]) / det4;

        return false;
    }

    inline bool mvie_v3(const double *v0,
                        const double *v1,
                        const double *v2,
                        double *L,
                        double *p)
    {
        const double o0 = (v0[0] + v1[0] + v2[0]) / 3.0;
        const double o1 = (v0[1] + v1[1] + v2[1]) / 3.0;

        const double m00 = 0.5 * (v0[0] + v1[0]) - o0;
        const double m01 = 0.5 * (v0[1] + v1[1]) - o1;
        const double m10 = 0.5 * (v1[0] + v2[0]) - o0;
        const double m11 = 0.5 * (v1[1] + v2[1]) - o1;
        const double m20 = 0.5 * (v2[0] + v0[0]) - o0;
        const double m21 = 0.5 * (v2[1] + v0[1]) - o1;

        const double det0 = m10 * m21 - m11 * m20;
        const double det1 = m20 * m01 - m21 * m00;
        const double det2 = m00 * m11 - m01 * m10;

        if (std::fabs(det0) < eps * eps ||
            std::fabs(det1) < eps * eps ||
            std::fabs(det2) < eps * eps)
        {
            return true;
        }

        const double det12 = det1 * det2;
        const double det02 = det0 * det2;
        const double det01 = det0 * det1;

        const double Q0 = (-m11 * m21 / det12) +
                          (-m01 * m11 / det01) +
                          (-m01 * m21 / det02);
        const double Q1 = 0.5 * ((m01 * m10 + m00 * m11) / det01 +
                                 (m01 * m20 + m00 * m21) / det02 +
                                 (m11 * m20 + m10 * m21) / det12);
        const double Q2 = (-m10 * m20 / det12) +
                          (-m00 * m10 / det01) +
                          (-m00 * m20 / det02);
        const double det_Q = Q0 * Q2 - Q1 * Q1;

        p[0] = o0;
        p[1] = o1;
        L[0] = std::sqrt(Q2 / det_Q);
        L[1] = -Q1 / (det_Q * L[0]);
        L[2] = std::sqrt(Q0 / det_Q - L[1] * L[1]);

        return false;

        // wxx
        const double f2_param = sqrt(3) / 6;
        const double f1_0 = 0.5 * (v0[0] - o0);
        const double f1_1 = 0.5 * (v0[1] - o1);
        const double f2_0 = f2_param * (v1[0] - v2[0]);
        const double f2_1 = f2_param * (v1[1] - v2[1]);

        const double L2_00 = f1_0 * f1_0 + f2_0 * f2_0;
        const double L2_01 = f1_0 * f1_1 + f2_0 * f2_1;
        const double L2_10 = f1_0 * f1_1 + f2_0 * f2_1;
        const double L2_11 = f1_1 * f1_1 + f2_1 * f2_1;

        std::cout << "****" << std::endl;
        std::cout << "wzp: " <<  L[0] << ", " << L[1] << ", " << L[2] << std::endl;
        L[0] = std::sqrt(L2_00);
        L[1] = L2_01 / L[0];
        L[2] = std::sqrt(L2_11 - L[1] * L[1]);
        std::cout << "wxx: " << L[0] << ", " << L[1] << ", " << L[2] << std::endl;
    }

    inline bool mvie_ccw_v4(const double *v0,
                            const double *v1,
                            const double *v2,
                            const double *v3,
                            double *L,
                            double *p)
    {
        const double sqr_d0 = (v0[0] - v1[0]) * (v0[0] - v1[0]) + (v0[1] - v1[1]) * (v0[1] - v1[1]);
        const double sqr_d1 = (v1[0] - v2[0]) * (v1[0] - v2[0]) + (v1[1] - v2[1]) * (v1[1] - v2[1]);
        const double sqr_d2 = (v2[0] - v3[0]) * (v2[0] - v3[0]) + (v2[1] - v3[1]) * (v2[1] - v3[1]);
        const double sqr_d3 = (v3[0] - v0[0]) * (v3[0] - v0[0]) + (v3[1] - v0[1]) * (v3[1] - v0[1]);

        double sqr_d = sqr_d0;
        int m = sqr_d < sqr_d1 ? 1 : 0;
        sqr_d = sqr_d < sqr_d1 ? sqr_d1 : sqr_d;
        m = sqr_d < sqr_d2 ? 2 : m;
        sqr_d = sqr_d < sqr_d2 ? sqr_d2 : sqr_d;
        m = sqr_d < sqr_d3 ? 3 : m;
        sqr_d = sqr_d < sqr_d3 ? sqr_d3 : sqr_d;

        const double *v;
        if (m == 1)
        {
            v = v0;
            v0 = v1;
            v1 = v2;
            v2 = v3;
            v3 = v;
        }
        else if (m == 2)
        {
            v = v3;
            v3 = v2;
            v2 = v0;
            v0 = v3;
            v3 = v1;
            v1 = v;
        }
        else if (m == 3)
        {
            v = v3;
            v3 = v2;
            v2 = v1;
            v1 = v0;
            v0 = v;
        }

        const double d0 = v1[0] - v0[0];
        const double d1 = v1[1] - v0[1];
        const double scale = std::sqrt(sqr_d);
        const double ct = d0 / scale;
        const double st = d1 / scale;

        const double r20 = (ct * (v2[0] - v0[0]) + st * (v2[1] - v0[1])) / scale;
        const double r21 = (-st * (v2[0] - v0[0]) + ct * (v2[1] - v0[1])) / scale;
        const double r30 = (ct * (v3[0] - v0[0]) + st * (v3[1] - v0[1])) / scale;
        const double r31 = (-st * (v3[0] - v0[0]) + ct * (v3[1] - v0[1])) / scale;

        const double u30 = r21 * r21;
        const double u31 = -2.0 * u30;
        const double u32 = 2.0 * r21 * (r20 - 1.0);
        const double u33 = u30;
        const double u34 = -u32;
        const double x0 = r20 * r31 - r21 * r30;
        const double x1 = r21 - r31;
        const double x2 = r30 - r20;
        const double u40 = x0 * x0;
        const double u41 = 2.0 * x0 * x1;
        const double u42 = 2.0 * x0 * x2;
        const double u43 = x1 * x1;
        const double u44 = 2.0 * x1 * x2;
        const double u53 = r31 * r31;
        const double u54 = -2.0 * r31 * r30;

        const double a00 = (u34 * u41 - u31 * u44) * u53 + (u31 * u43 - u33 * u41) * u54;
        const double b00 = (u32 * u41 - u31 * u42) * u53;
        const double a01 = -((u34 * u40 - u30 * u44) * u53 + (u30 * u43 - u33 * u40) * u54);
        const double b01 = -(u32 * u40 - u30 * u42) * u53;
        const double b02 = (u31 * u40 - u30 * u41) * u53;
        const double a11 = -(u31 * u40 - u30 * u41) * u54;
        const double a12 = b02;

        double c0 = -2.0 * a00 * (b00 + a01);
        double c1 = 2.0 * (b00 * (2.0 * a01 - b00) + a00 * (a11 - 2.0 * b01));
        double c2 = b00 * (2.0 * b01 - a11);
        const double cm = std::max(std::max(std::fabs(c0), std::fabs(c1)), std::fabs(c2));
        c0 /= cm;
        c1 /= cm;
        c2 /= cm;

        double sol = 0.0;
        if (std::fabs(c0) > eps)
        {
            const double delta = c1 * c1 - 4.0 * c0 * c2;
            if (delta < 0.0)
            {
                return true;
            }
            const double sqrt_delta = std::sqrt(delta);
            const double inv_db_c0 = 0.5 / c0;
            const double sol0 = (sqrt_delta - c1) * inv_db_c0;
            const double sol1 = (-sqrt_delta - c1) * inv_db_c0;
            sol = sol0 > 0.0 && sol0 < 1.0 ? sol0 : sol1;
        }
        else if (std::fabs(c1) > eps)
        {
            sol = -c2 / c1;
        }
        else
        {
            return true;
        }

        if (!(sol > 0.0 && sol < 1.0))
        {
            return true;
        }

        const double e00 = sol * sol;
        const double e01 = -sol;
        const double e02 = (sol / a12) * (-sol * a01 + a11 - b01);
        const double e11 = 1.0;
        const double e12 = (sol * (sol * a00 + b00 - a01) - b01) / a12;
        const double e22 = (sol * (sol * (a01 * a01 - a00 * a11) + 2.0 * a01 * b01 - a11 * b00) + b01 * b01) / (a12 * a12);

        const double det_e = e11 * e22 - e12 * e12;
        const double q0 = -scale * (e01 * e22 - e02 * e12) / det_e;
        const double q1 = -scale * (e02 * e11 - e01 * e12) / det_e;
        const double q_den = -((q0 * e01 + q1 * e02) + e00 * scale) * scale;

        const double Q0 = (ct * (ct * e11 - st * e12) - st * (ct * e12 - st * e22)) / q_den;
        const double Q1 = (st * (ct * e11 - st * e12) + ct * (ct * e12 - st * e22)) / q_den;
        const double Q2 = (st * (ct * e12 + st * e11) + ct * (ct * e22 + st * e12)) / q_den;
        const double det_Q = Q0 * Q2 - Q1 * Q1;

        p[0] = ct * q0 - st * q1 + v0[0];
        p[1] = st * q0 + ct * q1 + v0[1];
        L[0] = std::sqrt(Q2 / det_Q);
        L[1] = -Q1 / (det_Q * L[0]);
        L[2] = std::sqrt(Q0 / det_Q - L[1] * L[1]);

        return false;
    }

    inline bool mvie_ccw_v5(const double *v0,
                            const double *v1,
                            const double *v2,
                            const double *v3,
                            const double *v4,
                            double *L,
                            double *p)
    {
        const double o0 = 0.2 * (v0[0] + v1[0] + v2[0] + v3[0] + v4[0]);
        const double o1 = 0.2 * (v0[1] + v1[1] + v2[1] + v3[1] + v4[1]);

        const double w00 = v0[0] - o0;
        const double w01 = v0[1] - o1;
        const double w10 = v1[0] - o0;
        const double w11 = v1[1] - o1;
        const double w20 = v2[0] - o0;
        const double w21 = v2[1] - o1;
        const double w30 = v3[0] - o0;
        const double w31 = v3[1] - o1;
        const double w40 = v4[0] - o0;
        const double w41 = v4[1] - o1;

        const double det0 = w00 * w11 - w01 * w10;
        const double det1 = w10 * w21 - w11 * w20;
        const double det2 = w20 * w31 - w21 * w30;
        const double det3 = w30 * w41 - w31 * w40;
        const double det4 = w40 * w01 - w41 * w00;

        if (std::fabs(det0) < eps * eps ||
            std::fabs(det1) < eps * eps ||
            std::fabs(det2) < eps * eps ||
            std::fabs(det3) < eps * eps ||
            std::fabs(det4) < eps * eps)
        {
            return true;
        }

        const double u00 = (w11 - w01) / det0;
        const double u01 = (w00 - w10) / det0;
        const double u10 = (w21 - w11) / det1;
        const double u11 = (w10 - w20) / det1;
        const double u20 = (w31 - w21) / det2;
        const double u21 = (w20 - w30) / det2;
        const double u30 = (w41 - w31) / det3;
        const double u31 = (w30 - w40) / det3;
        const double u40 = (w01 - w41) / det4;
        const double u41 = (w40 - w00) / det4;

        const double d0 = u10 - u00;
        const double d1 = u11 - u01;
        const double scale = std::sqrt(d0 * d0 + d1 * d1);
        const double ct = d0 / scale;
        const double st = d1 / scale;

        const double r20 = (ct * (u20 - u00) + st * (u21 - u01)) / scale;
        const double r21 = (-st * (u20 - u00) + ct * (u21 - u01)) / scale;
        const double r30 = (ct * (u30 - u00) + st * (u31 - u01)) / scale;
        const double r31 = (-st * (u30 - u00) + ct * (u31 - u01)) / scale;
        const double r40 = (ct * (u40 - u00) + st * (u41 - u01)) / scale;
        const double r41 = (-st * (u40 - u00) + ct * (u41 - u01)) / scale;

        const double a00 = 2.0 * r21;
        const double a01 = 2.0 * r20 * r21;
        const double a02 = r21 * r21;
        const double a10 = 2.0 * r31;
        const double a11 = 2.0 * r30 * r31;
        const double a12 = r31 * r31;
        const double a20 = 2.0 * r41;
        const double a21 = 2.0 * r40 * r41;
        const double a22 = r41 * r41;
        const double b0 = r20 - r20 * r20;
        const double b1 = r30 - r30 * r30;
        const double b2 = r40 - r40 * r40;

        const double c00 = a11 * a22 - a12 * a21;
        const double c01 = a02 * a21 - a01 * a22;
        const double c02 = a01 * a12 - a02 * a11;
        const double c10 = a12 * a20 - a10 * a22;
        const double c11 = a00 * a22 - a02 * a20;
        const double c12 = a02 * a10 - a00 * a12;
        const double c20 = a10 * a21 - a11 * a20;
        const double c21 = a01 * a20 - a00 * a21;
        const double c22 = a00 * a11 - a01 * a10;

        const double det_a = a00 * c00 + a01 * c10 + a02 * c20;

        const double e00_o = 0.0;
        const double e01_o = -0.5;
        const double e11_o = 1.0;
        const double e02_o = (c00 * b0 + c01 * b1 + c02 * b2) / det_a;
        const double e12_o = (c10 * b0 + c11 * b1 + c12 * b2) / det_a;
        const double e22_o = (c20 * b0 + c21 * b1 + c22 * b2) / det_a;

        const double det_e_o = e11_o * e22_o - e12_o * e12_o;
        const double q0_o = -scale * (e01_o * e22_o - e02_o * e12_o) / det_e_o;
        const double q1_o = -scale * (e02_o * e11_o - e01_o * e12_o) / det_e_o;
        const double q_den_o = -((q0_o * e01_o + q1_o * e02_o) + e00_o * scale) * scale;

        const double p0_o = ct * q0_o - st * q1_o + u00;
        const double p1_o = st * q0_o + ct * q1_o + u01;

        const double Q0_o = (ct * (ct * e11_o - st * e12_o) - st * (ct * e12_o - st * e22_o)) / q_den_o;
        const double Q1_o = (st * (ct * e11_o - st * e12_o) + ct * (ct * e12_o - st * e22_o)) / q_den_o;
        const double Q2_o = (st * (ct * e12_o + st * e11_o) + ct * (ct * e22_o + st * e12_o)) / q_den_o;
        const double det_o = Q0_o * Q2_o - Q1_o * Q1_o;

        const double e00 = -1.0;
        const double e01 = p0_o;
        const double e02 = p1_o;
        const double e11 = -p0_o * p0_o + Q2_o / det_o;
        const double e12 = -p0_o * p1_o - Q1_o / det_o;
        const double e22 = -p1_o * p1_o + Q0_o / det_o;

        const double det_e = e11 * e22 - e12 * e12;
        const double q0 = -(e01 * e22 - e02 * e12) / det_e;
        const double q1 = -(e02 * e11 - e01 * e12) / det_e;
        const double q_den = -((q0 * e01 + q1 * e02) + e00);

        const double Q0 = e11 / q_den;
        const double Q1 = e12 / q_den;
        const double Q2 = e22 / q_den;
        const double det_Q = Q0 * Q2 - Q1 * Q1;

        p[0] = q0 + o0;
        p[1] = q1 + o1;
        L[0] = std::sqrt(Q2 / det_Q);
        L[1] = -Q1 / (det_Q * L[0]);
        L[2] = std::sqrt(Q0 / det_Q - L[1] * L[1]);

        return false;


        // wxx
        Eigen::Matrix<double, 2 ,5> v;
        v.col(0)[0] = v0[0];
        v.col(0)[1] = v0[1];
        v.col(1)[0] = v1[0];
        v.col(1)[1] = v1[1];
        v.col(2)[0] = v2[0];
        v.col(2)[1] = v2[1];
        v.col(3)[0] = v3[0];
        v.col(3)[1] = v3[1];
        v.col(4)[0] = v4[0];
        v.col(4)[1] = v4[1];

        // 这一小段本来作用是贴边然后降低计算量，但是下面偷懒只是拿来测试，所以这段用处不大
        for( int i = 0; i < 5; i++ )
        {
            v.col(i) = v.col(i) - Eigen::Vector2d{v0[0], v0[1]};
        }
        Eigen::Vector2d v_temp = (v.col(1)).normalized();
        Eigen::Matrix2d R;
        R << v_temp[0], v_temp[1],
            -v_temp[1], v_temp[0];
        for( int i = 0; i < 5; i++ )
        {
            v.col(i) = R * v.col(i);
        }

        // 线坐标计算
        Eigen::Matrix<double, 3 ,5> X;
        for( int i = 0; i < 5; i++ )
        {
            X.col(i)[0] = v.col(i)[0] * v.col((i+1)%5)[1] - v.col(i)[1] * v.col((i+1)%5)[0];
            X.col(i)[1] = v.col(i)[1] - v.col((i+1)%5)[1];
            X.col(i)[2] = v.col((i+1)%5)[0] - v.col(i)[0];
        }
        // 带入计算椭圆线方程
        Eigen::Matrix<double, 5 ,5> A_mat;
        Eigen::Matrix<double, 5 ,1> b;
        for( int i = 0; i < 5; i++ )
        {
            b[i] = -X.col(i)[0] * X.col(i)[0];
            A_mat.row(i)[0] = X.col(i)[0] * X.col(i)[1] * 2;
            A_mat.row(i)[1] = X.col(i)[0] * X.col(i)[2] * 2;
            A_mat.row(i)[2] = X.col(i)[1] * X.col(i)[1];
            A_mat.row(i)[3] = X.col(i)[1] * X.col(i)[2] * 2;
            A_mat.row(i)[4] = X.col(i)[2] * X.col(i)[2];
        }

        Eigen::Matrix<double, 5 ,1> Ellipse_line_vec = A_mat.fullPivLu().solve(b);
        Eigen::Matrix3d Ellipse_line;

        double D_line = Ellipse_line_vec[0];
        double E_line = Ellipse_line_vec[1];
        double A_line = Ellipse_line_vec[2];
        double B_line = Ellipse_line_vec[3];
        double C_line = Ellipse_line_vec[4];
        double F_line = 1.0;
        Ellipse_line << A_line, B_line, D_line,
                        B_line, C_line, E_line,
                        D_line, E_line, F_line;

        // 获得椭圆点方程
        Eigen::Matrix3d Ellipse_point = Ellipse_line.inverse();
        double A_point = Ellipse_point.col(0)[0];
        double B_point = Ellipse_point.col(1)[0];
        double C_point = Ellipse_point.col(1)[1];
        double D_point = Ellipse_point.col(2)[0];
        double E_point = Ellipse_point.col(2)[1];
        double F_point = Ellipse_point.col(2)[2];

        Eigen::Vector2d center_point;
        center_point[0] = -(B_point * E_point - D_point * C_point) / (B_point * B_point - A_point * C_point);
        center_point[1] = -(B_point * D_point - A_point * E_point) / (B_point * B_point - A_point * C_point);

        double N_point_neg =-F_point + (A_point * center_point[0] * center_point[0] + 
                                        C_point * center_point[1] * center_point[1] +
                                        B_point * center_point[0] * center_point[1] * 2);
        double N_point_neg_inv = 1 / N_point_neg;

        center_point = (R.transpose() * center_point + Eigen::Vector2d{v0[0], v0[1]}).eval();

        Eigen::Matrix2d LLT_inv;
        LLT_inv << A_point * N_point_neg_inv, B_point * N_point_neg_inv,
                   B_point * N_point_neg_inv, C_point * N_point_neg_inv;
        Eigen::Matrix2d LLT = LLT_inv.inverse();
        LLT = (R.transpose() * LLT * R).eval();
        Eigen::Vector3d L_wxx;
        L_wxx[0] = std::sqrt(LLT.col(0)[0]);
        L_wxx[1] = LLT.col(0)[1] / L_wxx[0];
        L_wxx[2] = std::sqrt(LLT.col(1)[1] - L_wxx[1] * L_wxx[1]);


        std::cout << "=====" << std::endl;
        std::cout << "=====" << std::endl;
        std::cout << "=====" << std::endl;
        std::cout << "p:" << std::endl;
        std::cout << "wzp: " << p[0] << ", " << p[1] << std::endl;
        std::cout << "wxx: " << center_point[0] << ", " << center_point[1] << std::endl;
        std::cout << "L:" << std::endl;
        std::cout << "wzp: " << L[0] << ", " << L[1] << ", " << L[2] << std::endl;
        std::cout << "wxx: " << L_wxx[0] << ", " << L_wxx[1] << ", " << L_wxx[2] << std::endl;
    }

    inline int add_h(const double *h0,
                     const double *h1,
                     const double *h,
                     double d,
                     const double *h_m)
    {
        const double det = h0[0] * h1[1] - h0[1] * h1[0];
        const double l0 = (h[0] * h1[1] - h[1] * h1[0]);
        const double l1 = (h[1] * h0[0] - h[0] * h0[1]);

        //
        if(det == 0)
        {
            if(h0[0] == h1[0] && h0[1] == h1[1])
            {
                if(h0[2] > h1[2])
                    return 1;
                else
                    return 0;
            }

            if(d == 3)
                return 2;
            else
            {
                /* code */
                const double l_m = (h_m[1] * h0[0] - h_m[0] * h0[1]);
                if( l_m * l1 < 0 )
                    return -1;
                else
                    return 2;
            }
        }
        if(l0 == 0)
        {
            if(h[0] == h1[0] && h[1] == h1[1])
            {
                if(h[2] > h1[2])
                    return 1;
                else
                    return 2;
            }
            return 0;
        }
        if(l1 == 0)
        {
            if(h[0] == h0[0] && h[1] == h0[1])
            {
                if(h[2] > h0[2])
                    return 0;
                else
                    return 2;
            }
            return 1;
        }

        const bool s0 = (det > 0.0 && l0 > 0.0) || (det < 0.0 && l0 < 0.0);
        const bool s1 = (det > 0.0 && l1 > 0.0) || (det < 0.0 && l1 < 0.0);

        if (!s0 && !s1)
        {
            // bounded
            return -1;
        }
        else if (s0 && !s1)
        {
            // h0 included
            return 0;
        }
        else if (!s0 && s1)
        {
            // h1 included
            return 1;
        }
        else
        {
            // h included
            return 2;
        }
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

    // w/s: bounded, w0, w1, w2, w3, w4
    template <int d>
    inline bool mvie2d_basis(const double *halves,
                             const int m,
                             const int w[6],
                             int s[6],
                             double e[5],
                             int *next,
                             int *prev)
    {
        // std::cout << "****" << std::endl;

        s[0] = w[0];
        s[1] = w[1];
        s[2] = w[2];
        s[3] = w[3];
        s[4] = w[4];
        s[5] = w[5];

        if (s[0])
        {
            if (d == 3)
            {
                double v0[2], v1[2], v2[2];
                const double *h0 = halves + s[1] * 3;
                const double *h1 = halves + s[2] * 3;
                const double *h2 = halves + s[3] * 3;
                if (h2v_3(h0, h1, h2, v0, v1, v2))
                {
                    return true;
                }
                if (mvie_v3(v0, v1, v2, e, e + 3))
                {
                    return true;
                }
            }
            if (d == 4)
            {
                double v0[2], v1[2], v2[2], v3[2];
                const double *h0 = halves + s[1] * 3;
                const double *h1 = halves + s[2] * 3;
                const double *h2 = halves + s[3] * 3;
                const double *h3 = halves + s[4] * 3;
                if (h2v_ccw_4(h0, h1, h2, h3, v0, v1, v2, v3))
                {
                    return true;
                }
                if (mvie_ccw_v4(v0, v1, v2, v3, e, e + 3))
                {
                    return true;
                }
            }
        }

        if (m == 0)
        {
            // std::cout << "????" << std::endl;

            return false;
        }

        int w_new[6] = {0};
        for (int i = 0; i != m; i = next[i])
        {
            // int nn = next[0];
            // std::cout << std::endl;

            // std::cout << nn << ", ";
            // for( int i = 0; i < 9; i++ )
            // {
            //     // std::cout << next[nn] << ", ";
            //     nn = next[nn];
            // }
            // std::cout << std::endl;

            // std::cout << "d= " << d << ", i= " << i << ", m= " << m << std::endl;

            const double *plane_i = halves + 3 * i;

            bool active = true;
            if (s[0])
            {
                const double r0 = plane_i[0] * e[0] + plane_i[1] * e[1];
                const double r1 = plane_i[1] * e[2];
                active = (std::sqrt(r0 * r0 + r1 * r1) +
                          plane_i[0] * e[3] +
                          plane_i[1] * e[4] +
                          plane_i[2]) > eps;
            }

            // std::cout << "s[0]= " << s[0] << ", ac= " << active << std::endl;
            // std::cout << "s[1]= " << s[1] << ", s[2]= " << s[2] << ", s[3]= " << s[3] << ", s[4]= " << s[4] << ", s[5]= " << s[5] << std::endl;

            if (active)
            {
                w_new[0] = w[0];
                w_new[1] = w[1];
                w_new[2] = w[2];
                w_new[3] = w[3];
                w_new[4] = w[4];
                w_new[5] = w[5];
                w_new[d + 1] = i;

                if (w[0] == 0 && d > 1)
                {
                    const double *h0 = halves + w[1] * 3;
                    const double *h1 = halves + w[d] * 3;
                    const double *h_m = halves + w[2] * 3;
                    const int state = add_h(h0, h1, plane_i, d+1, h_m);
                    if (state < 0)
                    {
                        w_new[0] = 1;
                    }
                    else if (state == 0)
                    {
                        w_new[1] = i;
                        w_new[d + 1] = w[d];
                        w_new[d] = w[1];
                    }
                    else if (state == 2)
                    {
                        w_new[d + 1] = w[d];
                        w_new[d] = i;
                    }
                // std::cout << "state= " << state << std::endl;   

                }

                // std::cout << "w_new[1]= " << w_new[1] << ", w_new[2]= " << w_new[2] << ", w_new[3]= " << w_new[3] << ", w_new[4]= " << w_new[4] << ", w_new[5]= " << w_new[5] << std::endl;

                if (mvie2d_basis<d + 1>(halves, i, w_new, s, e, next, prev))
                {
                    return true;
                }

                i = move_to_front(i, next, prev);
            }
            // std::cout << "i= " << i << ", next i= " << next[i] << std::endl;
            // std::cout << "m= " << m << std::endl;
        }
        return false;
    }

    template <>
    inline bool mvie2d_basis<5>(const double *halves,
                                const int m,
                                const int w[6],
                                int s[6],
                                double e[5],
                                int *next,
                                int *prev)
    {
        s[0] = w[0];
        s[1] = w[1];
        s[2] = w[2];
        s[3] = w[3];
        s[4] = w[4];
        s[5] = w[5];

        if (s[0])
        {
            double v0[2], v1[2], v2[2], v3[2], v4[2];
            const double *h0 = halves + s[1] * 3;
            const double *h1 = halves + s[2] * 3;
            const double *h2 = halves + s[3] * 3;
            const double *h3 = halves + s[4] * 3;
            const double *h4 = halves + s[5] * 3;
            if (h2v_ccw_5(h0, h1, h2, h3, h4, v0, v1, v2, v3, v4))
            {
                return true;
            }
            if (mvie_ccw_v5(v0, v1, v2, v3, v4, e, e + 3))
            {
                return true;
            }
        }

        return false;
    }

    inline void rand_permutation(const int n,
                                 int *p)
    {
        typedef std::uniform_int_distribution<int> rand_int;
        typedef rand_int::param_type rand_range;

        static std::mt19937_64 gen;
        static rand_int rdi(0, 1);

        int j, k;
        for (int i = 0; i < n; ++i)
        {
            p[i] = i;
        }
        for (int i = 0; i < n; ++i)
        {
            rdi.param(rand_range(0, n - i - 1));
            j = rdi(gen) + i;
            k = p[j];
            p[j] = p[i];
            p[i] = k;
        }
    }

    inline bool mvie2d(const Eigen::MatrixX2d &A,
                       const Eigen::VectorXd &b,
                       Eigen::Matrix2d &R,
                       Eigen::Vector2d &p,
                       Eigen::Vector2d &r)
                       // A·D^2·A^T = L·L^T
    {
        int nn = b.size();
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> halves(3, nn);
        const Eigen::VectorXd scale = A.rowwise().norm();
        halves.topRows<2>() = (A.array().colwise() / scale.array()).transpose();
        halves.bottomRows<1>() = (-b.array() / scale.array()).transpose();
        

        // std::vector<int> index_vec;
        // for( int i = 0; i < nn; i++ )
        // {
        //     for( int j = i+1; j < nn; j++ )
        //     {
        //         if(halves(0, i)==halves(0, j) && halves(1, i)==halves(1, j))
        //         {
        //             // 越小越远
        //             if( halves(2, i) > halves(2, j) )
        //                 index_vec.push_back(j);
        //             else
        //                 index_vec.push_back(i);
        //         }
        //     }
        // }
        // std::sort(index_vec.begin(), index_vec.end());
        // index_vec.erase(std::unique(index_vec.begin(), index_vec.end()), index_vec.end());

        // std::sort(index_vec.rbegin(), index_vec.rend());
        // for (int col : index_vec) {
        //     halves.block(0, col, halves.rows(), halves.cols() - col - 1) = halves.block(0, col + 1, halves.rows(), halves.cols() - col - 1);
        // }
        // halves.conservativeResize(halves.rows(), halves.cols() - index_vec.size());

        // std::cout << halves << std::endl;

        Eigen::Matrix2d L;
        L.setIdentity();
        p.setZero();
        const int n = halves.cols();
        if (n < 3)
        {
            return true;
        }
        // std::cout << "===========" << std::endl;
        // std::cout << "A:" << std::endl;
        // std::cout << A.transpose() << std::endl;
        // std::cout << "b:" << std::endl;
        // std::cout << b.transpose() << std::endl;

        Eigen::VectorXi perm(n - 1);
        Eigen::VectorXi next(n);
        Eigen::VectorXi prev(n + 1);
        rand_permutation(n - 1, perm.data());
        prev(0) = 0;
        next(0) = perm(0) + 1;
        prev(perm(0) + 1) = 0;
        for (int i = 0; i < n - 2; ++i)
        {
            next(perm(i) + 1) = perm(i + 1) + 1;
            prev(perm(i + 1) + 1) = perm(i) + 1;
        }
        next(perm(n - 2) + 1) = n;

        int w[6] = {0};
        int s[6] = {0};
        double e[5] = {0.0};

        if (mvie2d_basis<0>(halves.data(), n, w, s, e, next.data(), prev.data()))
        {
            return true;
        }
        if (s[0] == 0)
        {
            return true;
        }

        L.setZero();
        L(0, 0) = e[0];
        L(1, 0) = e[1];
        L(1, 1) = e[2];
        p(0) = e[3];
        p(1) = e[4];

        // output
        Eigen::Matrix2d LLT = L * L.transpose();
        Eigen::JacobiSVD<Eigen::Matrix2d, Eigen::FullPivHouseholderQRPreconditioner> svd(LLT, Eigen::ComputeFullU);
        const Eigen::Matrix2d U = svd.matrixU();
        const Eigen::Vector2d S = svd.singularValues();
        if (U.determinant() < 0.0)
        {
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            r(0) = std::sqrt(S(1));
            r(1) = std::sqrt(S(0));
        }
        else
        {
            R = U;
            r = S.cwiseSqrt();
        }

        return false;
    }

} // namespace mvie2d

#endif
