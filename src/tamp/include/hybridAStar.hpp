// #ifndef HYBRIDASTAR_HPP
// #define HYBRIDASTAR_HPP
#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include "voxel_map.hpp"

// 定义一个表示坐标的结构体
struct Point {
    double x, y;
    double yaw;
    Point(){
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
    }
    Point(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_){}
};

struct Input {
    double vel;
    double delta;
    Input(double vel_, double delta_) : vel(vel_), delta(delta_){}
    Input(){
        vel = 0.0;
        delta = 0.0;
    }
};

// 定义一个表示节点的结构体
struct Node {
    Point point;
    Input input;
    double g; // 从起始点到当前节点的实际代价
    double h; // 从当前节点到目标点的估计代价（启发式函数值）
    Node* parent; // 指向父节点的指针
    Node(){
        point = Point();
        input = Input();
        g = 0.0;
        h = 0.0;
        parent = nullptr;
    }
    Node(Point p, double g_, double h_, Node* parent_) : point(p), g(g_), h(h_), parent(parent_) {
        input = Input(0.0, 0.0);
    }

    // 计算节点的 f 值（f = g + h）
    double f() const {
        return g + h;
    }
};

// 自定义优先队列的比较函数
struct CompareNode {
    bool operator()(const Node* n1, const Node* n2) const {
        return n1->f() > n2->f();
    }
};

// A* 算法函数
inline std::vector<Node> astar(const VoxelMap* map_ptr,  Point start,  Point goal) {
    // 创建一个优先队列（按节点的 f 值进行排序）
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    // 创建一个哈希表，用于记录节点是否已经访问过
    double x_size = 24;  //   -12 < x < 12
    double y_size = 24;  //   -12 < y < 12
    double yaw_size = 6.28 / 0.2; // -3.14 < yaw < 3.14
    std::vector<std::vector<std::vector<Node*>>> visited(
        int(x_size), 
        std::vector<std::vector<Node*>>(int(y_size), 
        std::vector<Node*>(std::ceil(yaw_size), nullptr)));
    // 初始化起始节点
    Node* startNode = new Node(start, 0.0, std::sqrt(std::pow(goal.x - start.x, 2) + std::pow(goal.y - start.y, 2)), nullptr);
    openSet.push(startNode);
    // A* 算法主循环
    while (!openSet.empty()) {
        // 从优先队列中取出当前 f 值最小的节点
        Node* current = openSet.top();
        openSet.pop();
        // 如果当前节点为目标节点，则返回路径
        double dist_to_goal = std::sqrt((current->point.x - goal.x)*(current->point.x - goal.x) 
                + (current->point.y - goal.y) * (current->point.y - goal.y) );
        if ( dist_to_goal < 1.5) {
            std::vector<Node> path;
            while (current != nullptr) {
                path.push_back(*current);
                // path.push_back(Eigen::Vector3d(current->point.x,current->point.y,current->point.yaw));
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        // 将当前节点标记为已访问
        visited[static_cast<int>(current->point.x + x_size / 2.0 )]
        [static_cast<int>(current->point.y + y_size / 2.0)]
        [static_cast<int>(current->point.yaw + yaw_size / 2.0) ] = current;
        std::vector<Node*> neibors;
        int nsteps = 1;
        double wheel_base = 1.0;
        double max_delta = atan(wheel_base / 3.0);
        for(double vel = 0.5; vel < 2.0; vel += 0.5){
            for(double delta = -max_delta; delta<max_delta; delta += max_delta / 5.0){
                Node* neibor = new Node();
                neibor->point = current->point;
                for(int i=0; i<nsteps; i++)
                {
                    neibor->point.yaw +=  vel * tan(delta) / wheel_base;
                    neibor->point.x +=  vel * cos(neibor->point.yaw);
                    neibor->point.y +=  vel * sin(neibor->point.yaw);
                }
                neibor->input.vel = vel;
                neibor->input.delta = delta;
                neibors.push_back(neibor);
            }
        }
        // 遍历当前节点的相邻节点
        for(auto neighbor : neibors){
            double x = neighbor->point.x;
            double y = neighbor->point.y;
            double yaw = neighbor->point.yaw;
           const Eigen::Vector3d position(x,y,0);
            if (x >= -x_size/2.0 && x < x_size/2 
                && y >= -y_size/2 && y < y_size/2 
                && yaw >= -yaw_size / 2.0 && yaw_size / 2.0
                && map_ptr->query(position) == 0) {
                //线性插值做碰撞检测
                Eigen::Vector3d se(current->point.x, current->point.y, 0);
                Eigen::Vector3d ee(x,y,0.0);
                bool collision_flag = false;
                for(double t=0.2; t<1; t+=0.2){
                    Eigen::Vector3d ce = se * (1.0 - t) + ee * t;
                    if(map_ptr->query(ce) > 0){
                        collision_flag = 1;
                        break;
                    }
                }
                if(collision_flag){
                    continue;
                }
                // 计算相邻节点的实际代价
                double delta_g = std::sqrt(pow(neighbor->point.x - current->point.x, 2) 
                                         + pow(neighbor->point.y - current->point.y, 2));
                double newG = current->g + delta_g;
                // 检查相邻节点是否已经被访问过，或者是否在 openSet 中
                int idx = static_cast<int>(x + x_size / 2.0);
                int idy = static_cast<int>(y + y_size / 2.0);
                int idz = static_cast<int>(yaw + yaw_size / 2.0);
                
                if (visited[idx][idy][idz] == nullptr || newG < visited[idx][idy][idz]->g) {
                    // 创建相邻节点
                    // 如果相邻节点不在 openSet 中，则将其加入 openSet
                    neighbor->g = newG;
                    neighbor->h = std::sqrt(std::pow(goal.x - x, 2) + std::pow(goal.y - y, 2));
                    neighbor->parent = current;
                    if (visited[idx][idy][idz] == nullptr) {
                        openSet.push(neighbor);
                    }
                    // 更新相邻节点的代价信息
                    visited[idx][idy][idz] = neighbor;
                }
            }
        }
    }
    // 如果未找到路径，则返回空路径
    std::cout<<"no path "<<start.x<<" "<<start.y<<" "<<goal.x<<" "<<goal.y<<std::endl;
    return std::vector<Node>();
}
// #endif