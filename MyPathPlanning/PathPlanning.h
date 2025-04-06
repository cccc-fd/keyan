#ifndef MYPATHPLANNING_PATHPLANNING_H
#define MYPATHPLANNING_PATHPLANNING_H

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdint>

#include "backup/pso.h"
#include "args.h"
#include "result.h"
#include "obstacle.h"
#include "PSOAFSA.h"

using namespace std;

class PathPlanning {
private:
    // 起始坐标（2个）
    vector<int> start;
    // 目标位置（12个）
    vector<int> goal;
    // 目标，初值与 goal 相同（12个）
    vector<int> target;
    // 空间限制：左、右、下、上、高度
    vector<int> limits;
    // 障碍切面圆
    vector<Obstacle> obs;
    // 断点坐标  总路径长度  路径中有几个点在障碍物内  插值的x坐标  插值的y坐标
    Result res;
    // 随机数种子
    uint32_t seed;
    // AFSA步长
    int visual;

public:
    // 构造器
    PathPlanning();

    PathPlanning(const vector<int> &limits);

    PathPlanning(vector<int> start, vector<int> goal, vector<int> limits);

    // 重载运算符
    friend ostream &operator<<(ostream &os, const PathPlanning &pp);

    // 障碍信息
    void obs_info();

    // 添加圆形障碍
    void add_circle_obs(double x, double y, double r, double Kv);

    // 添加椭圆障碍
    void add_ellipse_obs(double x, double y, double theta, double a, double b, double Kv);

    // 添加多边形障碍
    void add_convex(double x, double y, vector<vector<int>> V, double Kv);

    // 删除指定障碍
    void remove_obs(size_t idx);

    // 一维差分
    vector<double> diff(const vector<double> &origin);

    // 计算路径惩罚值，修正航路
    tuple<double, int> path_penalty(const vector<Obstacle>& obs, vector<double> Px, vector<double> Py);

    // 计算代价
    tuple<double, int, vector<double>, vector<double>>
    calc_path_length(vector<double> agent_pos, Args &args);


    // 优化路径
    tuple<vector<double>, double, int, vector<double>, vector<double>>
    optimize(int n_pts, int pointNums, int n_pop, int epochs, double phi, double vel_fact, double rad,
             const string& interpolation_mode = "cubic", const string& conf_type = "RB");

    /*
     ******************************* getter and setter ******************************
     * */

    const vector<int> &getStart() const;

    void setStart(const vector<int> &start);

    const vector<int> &getGoal() const;

    void setGoal(const vector<int> &goal);

    const vector<int> &getTarget() const;

    void setTarget(const vector<int> &target);

    const vector<int> &getLimits() const;

    void setLimits(const vector<int> &limits);

    const vector<Obstacle> &getObs() const;

    void setObs(const vector<Obstacle> &obs);

    const Result &getRes() const;

    void setRes(const Result &res);

    uint32_t getSeed() const;

    void setSeed(uint32_t seed);

    int getVisual() const;

    void setVisual(int visual);
};


#endif //MYPATHPLANNING_PATHPLANNING_H
