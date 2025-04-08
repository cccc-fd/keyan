#ifndef MYPATHPLANNING_PATHPLANNING_H
#define MYPATHPLANNING_PATHPLANNING_H

#include <vector>
#include <cmath>
#include <iostream>
#include <string>

#include "pso.h"
#include "args.h"
#include "result.h"
#include "obstacle.h"

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

public:
    // 构造器
    PathPlanning();
    PathPlanning(const vector<int>& limits);
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

    // 优化路径
    tuple<vector<double>, double, int, vector<double>, vector<double>>
    optimize(int n_pts, int _ns, int n_pop, int epochs, int k, double phi, double vel_fact, string conf_type,
             const string &int_var, bool normalize, double rad, const string &finterp, vector<vector<int>> xinit);

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
};


#endif //MYPATHPLANNING_PATHPLANNING_H
