#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>

struct ZLQInMaterial{
	double soil; //kg
	double cup_soil; //kg
    ZLQInMaterial(){
        soil = 0.0;
        cup_soil = 0.0;
    }
};
struct ZLQOutMaterial{
	std::multiset<double> cup;//生成材料的时间 time, size = num
	std::multiset<double> brick;
    ZLQOutMaterial(){
        cup.clear();
        brick.clear();
    }
};

class DataPool {
public:
    // 获取单例实例的静态方法
    static DataPool& Instance() {
        static DataPool instance; // C++11的线程安全局部静态变量
        return instance;
    }
    // 禁用拷贝构造和赋值操作符
    DataPool(const DataPool&) = delete;
    DataPool& operator=(const DataPool&) = delete;

    void showMessage() {
        std::cout << "DataPool instance address: " << this << std::endl;
    }
    ZLQInMaterial zlq_in_material;
    ZLQOutMaterial zlq_out_material;
private:
    // 私有构造函数和析构函数，防止外部创建或销毁实例
    DataPool() {
        std::cout << "DataPool instance created." << std::endl;
    }
    ~DataPool() {
        std::cout << "DataPool instance destroyed." << std::endl;
    }
};