//
// Created by 15017 on 2024/5/27.
//

#ifndef MYPATHPLANNING_EXAMPLE_H
#define MYPATHPLANNING_EXAMPLE_H

#include <vector>
#include <chrono>

#include "data.h"
//#include "utils.h"
#include "result.h"
#include "PathPlanning.h"

using namespace std;

class Example {

private:
    int breakpointNum;  // 断点个数
    int pointNum;       // 插值点个数
    int particleNum;    // 粒子个数
    int epochs;         // 迭代次数
    string interpType;  // 插值方式
    int seed;           // 随机数种子
    int kv;             // 惩罚系数
    int visual;         // 步长

    PathPlanning pathPlanning;  // 一个航路规划的对象

    vector<Obstacle> obstacles; // 障碍信息
    vector<Result> Map_res; // 结果信息


public:

    // 无参构造
    Example();

    // 构造器，初始化迭代信息等等
    Example(int breakpointNum, int pointNum, int particleNum, int epochs, const string &interpType = "cubic");

    // 执行航路规划
    tuple<vector<vector<double>>, vector<Result>> run();


private:
    // 打印基本信息
    void printInformation();

    // 获取障碍信息
    vector<Obstacle> get_all_obs();

    // 获取切面
    tuple<double, double, double> get_circle(int x, int y, int z, int r, int h);

    // 获取切面
    tuple<double, double, double> get_mountain_circle(int x, int y, int z, int r, int h);

    // 获取切面
    tuple<double, double, double> get_weather_circle(int x, int y, int z, int r, int h);

    // 计算不规则区域的中心点位置
    tuple<double, double> centroid(const vector<vector<int>> &V);

    // 获取航路L2的长度
    double get_l2(const int &cycle_count, const double &goal_length);

    // 获取航路L3的长度
    double get_l3(vector<int> _goal, double l1, int cycle_count = 0, double goal_length = 0.0);

    // 计算需要扫描几次侦察目标才能完成侦察
    tuple<int, double> calcu_cycle_count(double F, int h, vector<int> _goal);



    // ******************************** getter and setter *************************
public:
    int getBreakpointNum() const;

    void setBreakpointNum(int breakpointNum);

    int getPointNum() const;

    void setPointNum(int pointNum);

    int getParticleNum() const;

    void setParticleNum(int particleNum);

    int getEpochs() const;

    void setEpochs(int epochs);

    const string &getInterpType() const;

    void setInterpType(const string &interpType);

    int getSeed() const;

    void setSeed(int seed);

    int getKv() const;

    void setKv(int kv);

    int getVisual() const;

    void setVisual(int visual);

    const vector<Obstacle> &getObstacles() const;

    void setObstacles(const vector<Obstacle> &obstacles);
};


#endif //MYPATHPLANNING_EXAMPLE_H
