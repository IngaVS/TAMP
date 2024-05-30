#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tools/gridmap.hpp>
#include <tools/config.hpp>
#include <tinycolormap.hpp>
using namespace std;

// pcl
pcl::PointCloud<pcl::PointXYZRGB> cloud_map;
pcl::PointCloud<pcl::PointXYZ> cloud_map_2d;
pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
pcl::PointXYZ sensor_pose;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
vector<Eigen::Vector2d> cors;

// random
random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_theta;
uniform_real_distribution<double> rand_radius;

// ros
ros::Publisher local_map_pub;
ros::Publisher global_map_pub, global_map_2d_pub;
ros::Publisher mesh_map_pub;
ros::Subscriber odom_sub;
ros::Timer sensor_timer;
sensor_msgs::PointCloud2 global_msg, global_msg_2d;
visualization_msgs::Marker mesh_msg;

// params
bool has_odom = false;
bool has_map = false;
vector<int> obs_num = {1, 1, 1};
double resolution = 0.1;
double size_x = 30.0;
double size_y = 30.0;
double min_width = 0.3;
double min_dis = 0.3;
double max_width = 0.8;
double sensor_rate = 1.0;
double sensor_range = 5.0;
Eigen::Vector2d start_pos, end_pos;

// laser
constexpr int LINE_NUM = 256;
double laser_res = 2.0 * M_PI / LINE_NUM;
Eigen::VectorXi idx_map = Eigen::VectorXi::Constant(LINE_NUM, -1);
Eigen::VectorXd dis_map = Eigen::VectorXd::Constant(LINE_NUM, 9999.0);

bool crossBoolean2(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return (a(0)*b(1)-b(0)*a(1) > 0);
}

pcl::PointCloud<pcl::PointXYZ> fillConvexPolygon(vector<Eigen::Vector2d> poly_vs)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

    if (poly_vs.size() < 3)
        return cloud_polygon;
    
    double down = 9999.0;
    double up = -9999.0;
    double left = 9999.0;
    double right = -9999.0;
    
    // AABB box
    for (size_t i=0; i<poly_vs.size(); i++)
    {
        if (poly_vs[i][0] > right)
            right = poly_vs[i][0];
        if (poly_vs[i][0] < left)
            left = poly_vs[i][0];
        if (poly_vs[i][1] > up)
            up = poly_vs[i][1];
        if (poly_vs[i][1] < down)
            down = poly_vs[i][1];
    }

    for (double x=left; x<right+resolution; x+=resolution)
    {
        for (double y=down; y<up+resolution; y+=resolution)
        {
            bool in_poly = false;
            Eigen::Vector2d O(x, y);

            for (size_t i=0; i<poly_vs.size() - 2; i++)
            {
                // if a point is in triangle
                Eigen::Vector2d A = poly_vs[0];
                Eigen::Vector2d B = poly_vs[i+1];
                Eigen::Vector2d C = poly_vs[i+2];
                if (crossBoolean2(B-A, O-A) && \
                    crossBoolean2(C-B, O-B) && \
                    crossBoolean2(A-C, O-C) )
                {
                    in_poly = true;
                    break;
                }                
            }

            if (in_poly)
            {
                pcl::PointXYZ pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.0;
                cloud_polygon.push_back(pt);
            }
        }
    }
    
    return cloud_polygon;
}

pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> generatePolygon(int K)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

    rand_w = uniform_real_distribution<double>(min_width, max_width);
    rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

    double radius = rand_w(eng);
    double theta = rand_theta(eng);
    double angle_res = 2.0 * M_PI / K;
    double small_r = radius * sin(angle_res/2.0);

    rand_radius = uniform_real_distribution<double>(-small_r, small_r);

    vector<Eigen::Vector2d> vs;
    for (int i=0; i<K; i++)
    {
        double a = angle_res * i + theta;
        double delta_theta = rand_theta(eng);
        double delta_radius = rand_radius(eng);
        Eigen::Vector2d p(cos(a)*radius + cos(a+delta_theta)*delta_radius, \
                          sin(a)*radius + sin(a+delta_theta)*delta_radius);
        vs.push_back(p);
    }
    cloud_polygon = fillConvexPolygon(vs);

    return std::make_pair(vs, cloud_polygon);
}
pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> generateRec()
{
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

    rand_w = uniform_real_distribution<double>(min_width, min_width);
    rand_h = uniform_real_distribution<double>(max_width, max_width);
    double w = rand_w(eng);
    double h = rand_h(eng);
    vector<Eigen::Vector2d> vs;
    vs.emplace_back(-w/2.0, h/2.0);
    vs.emplace_back(-w/2.0, -h/2.0);

    vs.emplace_back(w/2.0, -h/2.0);
    vs.emplace_back(w/2.0, h/2.0);

    cloud_polygon = fillConvexPolygon(vs);

    return std::make_pair(vs, cloud_polygon);
}  
bool checkDis(double x, double y){
    bool result = false;
    Eigen::Vector2d tmp(x, y);
    for(const auto pt : cors){
        if((tmp-pt).norm() < min_dis){
            result = true;
            return result;
        }
    }
    return result;
}
void generateMap()
{
   // ROS_WARN("begin to generate map!");
    cors.clear();
    cloud_map.points.clear();
    mesh_msg.points.clear();





    pcl::PointXYZ pt_random;
    //add rectangle
    uniform_real_distribution<double> rand_ColorRow = uniform_real_distribution<double>(-1, 1);//hanhan
    double iscol = rand_ColorRow(eng);
    // ROS_ERROR("1111111111111111111111111111111");
    // std::cout << "iscol = " << iscol << std::endl;
    
    // uniform_int_distribution<int> gap = uniform_int_distribution<int>(1, 3);
    // uniform_real_distribution<double> half = uniform_real_distribution<double>(0.6, 0.6);

    // if(iscol>0){
    //     int last_gapnum = -1;
    //     for(double rx = -6.0; rx<=6.1; ){
    //         int gap_num = gap(eng);
    //         if(gap_num==last_gapnum||(last_gapnum+gap_num)%2==0){
    //             continue;
    //         }
    //         else{
    //             last_gapnum = gap_num;
    //         }
    //         double half_l = half(eng);
    //         std::cout << "gap:nunm"<<gap_num<<std::endl;
    //         double last_y = 10.0;
    //         for(int idx = gap_num; idx >= 1; idx--){
    //             double center = 1.0*idx*20.0 / (1.0*(gap_num+1))-10.0;
    //             cors.push_back(Eigen::Vector2d(rx, center));
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(rx, last_y);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(rx, center+half_l);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(rx+0.2, center+half_l);//?
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(rx+0.2, last_y);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //             }
    //             last_y = center-half_l;
    //         }
    //         {
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(rx, last_y);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(rx, -10.0);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(rx+0.2, -10.0);//?
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(rx+0.2, last_y);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //             }
    //         }
    //         rx+=3.0; 
    //     }
    // }
    // else
    // {   
    //     int last_gapnum = -1;
    //     for(double ry = -6.0; ry<=6.1;){
    //         int gap_num = gap(eng);
    //         if(gap_num==last_gapnum||(last_gapnum+gap_num)%2==0){
    //             continue;
    //         }
    //         else{
    //             last_gapnum = gap_num;
    //         }
    //         double half_l = half(eng);
    //         double last_x = -10.0;
    //         for(int idx = 1; idx <= gap_num; idx++){
    //             double center = 1.0*idx*20.0 / (1.0*(gap_num+1))-10.0;
    //             cors.push_back(Eigen::Vector2d(center, ry));

    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(last_x, ry+0.2);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(last_x, ry);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(center-half_l, ry);
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(center-half_l, ry+0.2);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //             }
    //             last_x = center+half_l;
    //         }   
    //             {
    //                 pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //                 vector<Eigen::Vector2d> vs;
    //                 Eigen::Vector2d p1(last_x, ry+0.2);
    //                 vs.push_back(p1);
    //                 Eigen::Vector2d p2(last_x, ry);
    //                 vs.push_back(p2);
    //                 Eigen::Vector2d p3(10.0, ry);
    //                 vs.push_back(p3);
    //                 Eigen::Vector2d p4(10.0, ry+0.2);
    //                 vs.push_back(p4);
    //                 cloud_polygon = fillConvexPolygon(vs);
    //                 for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //                 {
    //                     pt_random.x = cloud_polygon.points[i].x;
    //                     pt_random.y = cloud_polygon.points[i].y;
    //                     pt_random.z = 0.0;
    //                     cloud_map.points.push_back(pt_random);
    //                     // cors.push_back(Eigen::Vector2d(pt_random.x, pt_random.y));
    //                 }
    //             }
    //              ry+=3.0; 
    //     }
    // }








    // if(iscol>0){
    //     for(double rx = -6.0; rx<=6.1; rx+=3.0){
    //         uniform_real_distribution<double> centergene = uniform_real_distribution<double>(-7.0, 7.0);
    //         double center = centergene(eng);
    //         double half_l = 0.65;
    //         cors.push_back(Eigen::Vector2d(rx, center));
    //         {
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(rx, 10.0);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(rx, center+half_l);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(rx+0.2, center+half_l);//?
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(rx+0.2, 10.0);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
                 
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //             }
    //         }
    //         {
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(rx, center-half_l);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(rx, -10.0);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(rx+0.2, -10.0);//?
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(rx+0.2,center-half_l);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //             }
    //         }

    //     }
    // }
    // else
    // {   

    //     for(double ry = -6.0; ry<=6.1;ry+=3.0){
    //         uniform_real_distribution<double> centergene = uniform_real_distribution<double>(-7.0, 7.0);
    //         double center = centergene(eng);
    //         double half_l = 0.65;
    //         cors.push_back(Eigen::Vector2d(center, ry));
    //         {
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(-10.0, ry+0.2);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(-10.0, ry);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(center-half_l, ry);
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(center-half_l, ry+0.2);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //                 // cors.push_back(Eigen::Vector2d(pt_random.x, pt_random.y));
    //             }
    //         }
    //                     {
    //             pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //             vector<Eigen::Vector2d> vs;
    //             Eigen::Vector2d p1(center+half_l, ry+0.2);
    //             vs.push_back(p1);
    //             Eigen::Vector2d p2(center+half_l, ry);
    //             vs.push_back(p2);
    //             Eigen::Vector2d p3(10.0, ry);
    //             vs.push_back(p3);
    //             Eigen::Vector2d p4(10.0, ry+0.2);
    //             vs.push_back(p4);
    //             cloud_polygon = fillConvexPolygon(vs);
    //             for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //             {
    //                 pt_random.x = cloud_polygon.points[i].x;
    //                 pt_random.y = cloud_polygon.points[i].y;
    //                 pt_random.z = 0.0;
    //                 cloud_map.points.push_back(pt_random);
    //                 // cors.push_back(Eigen::Vector2d(pt_random.x, pt_random.y));
    //             }
    //         }
    //     }
    // }
   
   
   // rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
    // rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);//hanhan

    rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x/2.0);
    rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y/2.0);//hanhan
    // generate polygon obs
    for (int k = 0; k<obs_num.size(); k++)
    {
        for (int i = 0; i < obs_num[k]; i++) 
        {
            // wxx
            double x, y;
            x = rand_x(eng);
            y = rand_y(eng);
            //-9.5 5 -2.5 9.5    4.5 5   -2.5 0.5
            if (sqrt(pow(x-start_pos[0], 2) + pow(y-start_pos[1], 2)) < 3.0||
                sqrt(pow(x-end_pos[0], 2) + pow(y-end_pos[1], 2)) < 3.0)
            {
                i--;
                continue;
            }

            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;
            if(checkDis(x, y)){
                i--;
                continue;
            }
            cors.push_back(Eigen::Vector2d(x, y));
            pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon = generatePolygon(k+3);
            for (size_t p=0; p<cloud_polygon.second.points.size(); p++)
            {
                pt_random.x = std::max(std::min(cloud_polygon.second.points[p].x + x, size_x / 2.0),-size_x / 2.0 );
                pt_random.y = std::max(std::min(cloud_polygon.second.points[p].y + y, size_y / 2.0),-size_y / 2.0 );
                pt_random.z = 0.0;
                pcl::PointXYZRGB pt_color;
                pt_color.x = pt_random.x;
                pt_color.y = pt_random.y;
                pt_color.z = pt_random.z;

                cloud_map.points.push_back(pt_color);
            }

            vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
            geometry_msgs::Point init_p;
            init_p.x = vector_polygon[0].x() + x;
            init_p.y = vector_polygon[0].y() + y;
            init_p.z = 0.0;
            for (int m=1; m<k+2; m++)
            {
                mesh_msg.points.push_back(init_p);
                geometry_msgs::Point p;
                p.x = vector_polygon[m].x() + x;
                p.y = vector_polygon[m].y() + y;
                p.z = 0.0;
                mesh_msg.points.push_back(p);
                p.x = vector_polygon[m+1].x() + x;
                p.y = vector_polygon[m+1].y() + y;
                p.z = 0.0;
                mesh_msg.points.push_back(p);
            }
        }
    }
    //not clock-wise


    //-9.5 5 -2.5 9.5    4.5 5   -2.5 0.5

//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-10, 5.6);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-10, 5.4);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-8.8, 5.4);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-8.8, 5.6);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-10, 4.6);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-10, 4.4);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-8.8, 4.4);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-8.8, 4.6);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     //-9.5 5 -2.5 9.5    4.5 5   -2.5 0.5
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-2.5-0.6, 10.0);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-2.5-0.6, 8.8);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-2.5-0.4, 8.8);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-2.5-0.4, 10.0);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-2.5+0.4, 10.0);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-2.5+0.4, 8.8);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-2.5+0.6, 8.8);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-2.5+0.6, 10.0);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }

//     //-9.5 5 -2.5 9.5    4.5 5   -2.5 0.5

//    {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(5.0-1.2, 5+0.6);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(5.0-1.2, 5+0.4);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(5.0, 5+0.4);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(5.0, 5+0.6);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(5.0-1.2, 5-0.4);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(5.0-1.2, 5-0.6);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(5.0,5-0.6);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(5.0, 5-0.4);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     //-9.5 5 -2.5 9.5    4.5 5   -2.5 0.5
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-2.5-0.6, 1.2);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-2.5-0.6, 0);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-2.5-0.4, 0);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-2.5-0.4, 1.2);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }
//     {
//         pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
//         vector<Eigen::Vector2d> vs;
//         Eigen::Vector2d p1(-2.5+0.4, 1.2);
//         vs.push_back(p1);
//         Eigen::Vector2d p2(-2.5+0.4, 0);
//         vs.push_back(p2);
//         Eigen::Vector2d p3(-2.5+0.6, 0);
//         vs.push_back(p3);
//         Eigen::Vector2d p4(-2.5+0.6, 1.2);
//         vs.push_back(p4);
//         cloud_polygon = fillConvexPolygon(vs);
//         for (size_t i=0; i<cloud_polygon.points.size(); i++)
//         {
//             pt_random.x = cloud_polygon.points[i].x;
//             pt_random.y = cloud_polygon.points[i].y;
//             pt_random.z = 0.0;
//             cloud_map.points.push_back(pt_random);
//         }
//     }

    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-1.998, 3.00);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-2.0000, 2.048);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(-1.01, 2.012);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(-1.018, 3.01);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }

    // }
    
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-10.2, 10);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-10.2, 0);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(-10.1,0);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(-10.1, 10);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }

    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-10, -0.1);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-10, -0.2);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(5,-0.2);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(5, -0.1);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }


    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(5.1, 10);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(5.1, 0);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(5.2,0);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(5.2, 10);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-10, 10.2);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-10, 10.1);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(5,10.1);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(5, 10.2);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-10, -9.9);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-10, -10);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(10.0, -10);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(10, -9.9);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }

    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(9.9, 10);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(9.9, -10);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(10, -10);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(10, 10);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(-10, 10);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(-10, 9.9);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(10, 9.9);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(10, 10);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }
    // }

    // ROS_WARN("Finish  generating map!");

    //     vector<Eigen::Vector2d> vector_polygon = vs;
    //     geometry_msgs::Point init_p;
    //     init_p.x = vector_polygon[0].x();
    //     init_p.y = vector_polygon[0].y();
    //     init_p.z = 0.0;
    //     for (int i=1; i<3; i++)
    //     {
    //         mesh_msg.points.push_back(init_p);
    //         geometry_msgs::Point p;
    //         p.x = vector_polygon[i].x();
    //         p.y = vector_polygon[i].y();
    //         p.z = 0.0;
    //         mesh_msg.points.push_back(p);
    //         p.x = vector_polygon[i+1].x();
    //         p.y = vector_polygon[i+1].y();
    //         p.z = 0.0;
    //         mesh_msg.points.push_back(p);
    //     }

    // }
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    //     vector<Eigen::Vector2d> vs;
    //     Eigen::Vector2d p1(7, 20);
    //     vs.push_back(p1);
    //     Eigen::Vector2d p2(6, 20);
    //     vs.push_back(p2);
    //     Eigen::Vector2d p3(6, 4);
    //     vs.push_back(p3);
    //     Eigen::Vector2d p4(7, 4);
    //     vs.push_back(p4);
    //     cloud_polygon = fillConvexPolygon(vs);
    //     for (size_t i=0; i<cloud_polygon.points.size(); i++)
    //     {
    //         pt_random.x = cloud_polygon.points[i].x;
    //         pt_random.y = cloud_polygon.points[i].y;
    //         pt_random.z = 0.0;
    //         cloud_map.points.push_back(pt_random);
    //     }

    //     vector<Eigen::Vector2d> vector_polygon = vs;
    //     geometry_msgs::Point init_p;
    //     init_p.x = vector_polygon[0].x();
    //     init_p.y = vector_polygon[0].y();
    //     init_p.z = 0.0;
    //     for (int i=1; i<3; i++)
    //     {
    //         mesh_msg.points.push_back(init_p);
    //         geometry_msgs::Point p;
    //         p.x = vector_polygon[i].x();
    //         p.y = vector_polygon[i].y();
    //         p.z = 0.0;
    //         mesh_msg.points.push_back(p);
    //         p.x = vector_polygon[i+1].x();
    //         p.y = vector_polygon[i+1].y();
    //         p.z = 0.0;
    //         mesh_msg.points.push_back(p);
    //     }



    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 1.0;
    mesh_msg.header.frame_id = "world";
}

void rcvOdomCallBack(const nav_msgs::OdometryConstPtr msg)
{
    sensor_pose.x = msg->pose.pose.position.x;
    sensor_pose.y = msg->pose.pose.position.y;
    sensor_pose.z = 0.0;
    has_odom = true;
}

void sensorCallback(const ros::TimerEvent &e)
{
    // if (!has_map || !has_odom)
    if (!has_map)
        return;
    static int count = 0;
    count++;
    
    global_map_pub.publish(global_msg);
    global_map_2d_pub.publish(global_msg_2d);
    mesh_map_pub.publish(mesh_msg);
    
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "random_map_node");
    ros::NodeHandle nh("~");

    bool use_pcd = false;
    bool use_pcd_2d = false;
    bool update_pcd = false;
    std::string pcd_file, pcd_2d_file, new_pcd_file;
    double z_min = 0.0;
    double z_max = 1.0;
    double map_scale = 1.0;

    nh.param<std::vector<int>>("map/obs_num", obs_num, std::vector<int>());
	nh.getParam("map/resolution", resolution);
	nh.getParam("map/size_x", size_x);
	nh.getParam("map/size_y", size_y);
	nh.getParam("map/min_width", min_width);
	nh.getParam("map/max_width", max_width);
    nh.getParam("map/min_dis", min_dis);
	nh.getParam("map/sensor_rate", sensor_rate);
	nh.getParam("map/sensor_range", sensor_range);

	nh.getParam("map/use_pcd_2d", use_pcd_2d);
	nh.getParam("map/pcd_2d_file", pcd_2d_file);
	nh.getParam("map/use_pcd", use_pcd);
	nh.getParam("map/pcd_file", pcd_file);
	nh.getParam("map/update_pcd", update_pcd);
	nh.getParam("map/new_pcd_file", new_pcd_file);
	nh.getParam("map/z_min", z_min);
	nh.getParam("map/z_max", z_max);
	nh.getParam("map/map_scale", map_scale);
	nh.getParam("map/has_odom", has_odom);

    std::vector<double> start_vec;
    std::vector<double> end_vec;
    nh.getParam("start_vec", start_vec);
    nh.getParam("end_vec", end_vec);
    start_pos = Eigen::Vector2d{start_vec[0], start_vec[1]};
    end_pos   = Eigen::Vector2d{  end_vec[0],   end_vec[1]};


    odom_sub  = nh.subscribe("odom", 1000, rcvOdomCallBack);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    global_map_2d_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud_2d", 1);
    mesh_map_pub = nh.advertise<visualization_msgs::Marker>("mesh_obstacles", 1);
    generateMap();

    if( use_pcd_2d )
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ> in_map;
        std::cout << "load 2d" << std::endl;
        reader.read<pcl::PointXYZ>(pcd_2d_file, in_map);
        std::cout << "start 2d" << std::endl;
        cloud_map_2d.points.clear();
        for( int ii = 0; ii < in_map.points.size(); ii++ )
        {
            // if( in_map.points[ii].z > z_min && in_map.points[ii].z < z_max )
            {
                pcl::PointXYZ pt = in_map.points[ii];
                pt.x = pt.x * map_scale;
                pt.y = pt.y * map_scale;
                // pt.z = 0;
                cloud_map_2d.points.push_back(pt);
            }
        }
        std::cout << "ok 2d" << std::endl;
    }

    if( use_pcd )
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ> in_map;
        std::cout << "load" << std::endl;
        reader.read<pcl::PointXYZ>(pcd_file, in_map);
        std::cout << "start" << std::endl;
        cloud_map.points.clear();
        for( int ii = 0; ii < in_map.points.size(); ii++ )
        {
            if( in_map.points[ii].z > z_min && in_map.points[ii].z < z_max )
            {
                pcl::PointXYZ pt = in_map.points[ii];
                pcl::PointXYZRGB pt_color;
                pt_color.x = pt.x * map_scale;
                pt_color.y = pt.y * map_scale;
                pt_color.z = pt.z * map_scale;
                const tinycolormap::Color color = tinycolormap::GetColor(( (pt.z - z_min) / (z_max - z_min) ), tinycolormap::ColormapType::Inferno);
                pt_color.r = color.r() * 255;
                pt_color.g = color.g() * 255;
                pt_color.b = color.b() * 255;

                cloud_map.points.push_back(pt_color);
            }
        }
        std::cout << "ok" << std::endl;
    }



    sensor_timer = nh.createTimer(ros::Duration(1.0/sensor_rate), sensorCallback);

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

    cloud_map_2d.width = cloud_map_2d.points.size();
    cloud_map_2d.height = 1;
    cloud_map_2d.is_dense = true;

    pcl::toROSMsg(cloud_map_2d, global_msg_2d);
    global_msg_2d.header.frame_id = "world";

    if( update_pcd )
    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGB>(new_pcd_file, cloud_map);
    }


	ros::spin();

    return 0;
}