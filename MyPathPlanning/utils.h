#ifndef MYPATHPLANNING_UTILS_H
#define MYPATHPLANNING_UTILS_H

#include <tuple>
#include <vector>
#include <random>
#include <fstream>
#include <Python.h>

#include "data.h"
#include "args.h"
#include "result.h"
#include "obstacle.h"
#include "Spline.h"

using namespace std;
using namespace SplineSpace;

// 球：获得切面圆半径
tuple<double, double, double> get_circle(int x, int y, int z, int r, int h);

// 山体：抽象为圆锥
tuple<double, double, double> get_mountain_circle(int x, int y, int z, int r, int h);

// 天气：抽象为圆柱
tuple<double, double, double> get_weather_circle(int x, int y, int z, int r, int h);

// 计算多边形的中心点
tuple<double, double> centroid(const vector<vector<int>> &V);


vector<double> build_Xinit(vector<double> &_start, vector<double> &_goal, int n_pts);


/*
 ********************************* General tool begin **********************************
 * */
// 获取所有障碍切面
vector<Obstacle> get_all_obs();

/*
 ****************************** utils for main.cpp begin *********************************
 * */
// 添加一个圆型障碍（主要是对 name 字段进行加工）
Obstacle add_circle_obs(double x, double y, double r, double Kv);

// 添加一个不规则障碍（主要对 name 字段进行加工）
Obstacle add_convex(double x, double y, vector<vector<int>> V, double kv);

// 计算起飞点与目标点的直线距离
vector<vector<double>> calcu_line_start_target(vector<vector<int>> &starts, vector<vector<int>> &targets);
// ****************************** utils for main.cpp end **********************************************


/*
 ********************************* run_example begin***************************************
 * */
// 扫描长度：l2
double get_l2(const int &cycle_count, const double &goal_length);

// 得到返航的长度：l3
double get_l3(vector<int> _goal, double l1, int cycle_count = 0, double goal_length = 0.0);

// 计算无人机探查目标需要往复几次实现全覆盖：目标长度
tuple<int, double> calcu_cycle_count(double F, int h, vector<int> _goal);
// ****************************** run_example end **********************************************

// 输出数据
void outResult(const vector<Result>& res);

// 插值 python解释器
vector<double> interpolation(const vector<double>& nums, int pointNums);

// 获取模拟数据
vector<vector<double>> getData();

#endif //MYPATHPLANNING_UTILS_H
