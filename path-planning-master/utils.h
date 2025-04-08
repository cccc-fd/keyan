#ifndef MYPATHPLANNING_UTILS_H
#define MYPATHPLANNING_UTILS_H

#include <tuple>
#include <vector>
#include <random>

#include "data.h"
#include "args.h"
#include "obstacle.h"
#include "utils_python.h"
#include "Spline.h"

//#include <matplotlibcpp.h>

//namespace plt = matplotlibcpp;

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

//
tuple<double, int> path_penalty(const vector<Obstacle>& obs, vector<double> Px, vector<double> Py);

// 一维数组拷贝
void deepcopy(const vector<double> &origin, vector<double> &target);

// 二维数组拷贝，类型：vector<vector<double>>
void deepcopy(const vector<vector<double>> &origin, vector<vector<double>> &vec_target);
// ******************************* General tool end **********************************************




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


/*
 ************************************** PSO *********************************************
 * */
// 对出界的粒子使用速度反向限制
vector<vector<double>> random_back_conf(const vector<vector<double>> &agent_vel);

// 对出界粒子速度应用双曲约束
vector<vector<double>>
hyperbolic_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB, vector<int> LB);

// 混合约束
vector<vector<double>>
mixed_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB, vector<int> LB);

// 计算cost，以觅食、聚群、追尾三种方式，返回三种方式中最好的pos和cost
tuple<vector<double>, vector<vector<double>>, vector<double>>
path_length(const vector<vector<double>>& agent_pos, const Args& args);

//
tuple<vector<double>, int> path_penalty(const vector<Obstacle>& obs, vector<vector<double>> Px, vector<vector<double>> Py);

// 返回最小化的函数，即当没有任何障碍冲突时的路径长度。
vector<double> calc_path_length(vector<vector<double>> agent_pos, const Args& args);

// 觅食行为
tuple<vector<vector<double>>, vector<double>>
forage(const vector<vector<double>>& agent_pos, const vector<double>& agent_cost, const Args& args, int Visual,
       int trynum);

// 聚群行为
tuple<vector<vector<double>>, vector<double>>
huddle(vector<vector<double>> agent_pos, const Args& args, const vector<vector<double>>& forage_agent_pos,
       vector<double> forage_cost);

// 追尾行为
tuple<vector<vector<double>>, vector<double>>
follow(const vector<vector<double>>& agent_pos, const Args& args, vector<vector<double>> huddle_agent_pos,
       const vector<double> &huddle_cost, const vector<double>& f, int Visual = 200);


vector<vector<double>> moveRandomly(const vector<vector<double>> &agent_pos, double Visual);

void printCost(const vector<double>& cost);

// ********************************** PSO end ******************************************



tuple<double, int, vector<double>, vector<double>>
calc_path_length(vector<double> agent_pos, Args &args);

// 插值
vector<double> interpolation(const vector<double>& nums, int pointNums);

// ********************************** matplotlib-cpp begin *****************************

vector<vector<double>> getData();

#endif //MYPATHPLANNING_UTILS_H
