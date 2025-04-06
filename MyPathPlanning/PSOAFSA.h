#ifndef MYPATHPLANNING_PSOAFSA_H
#define MYPATHPLANNING_PSOAFSA_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <random>
#include <algorithm>
#include <limits>

#include "args.h"
#include "utils.h"


using namespace std;

class PSOAFSA {

private:

    int visual;

    // engine 随机引擎
    default_random_engine engine;

    // 对出界的粒子使用速度反向限制
    vector<vector<double>> random_back_conf(const vector<vector<double>> &agent_vel);

    // 对出界粒子速度应用双曲约束
    vector<vector<double>>
    hyperbolic_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB,
                    vector<int> LB);

    // 混合约束
    vector<vector<double>>
    mixed_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB, vector<int> LB);

    // 计算cost，以觅食、聚群、追尾三种方式，返回三种方式中最好的pos和cost
    tuple<vector<double>, vector<vector<double>>, vector<double>>
    path_length(const vector<vector<double>> &agent_pos, const Args &args);

    // 修正路径，对已有的不满足约束的航路增加惩罚值，使之满足条件
    tuple<vector<double>, int>
    path_penalty(const vector<Obstacle> &obs, vector<vector<double>> Px, vector<vector<double>> Py);

    // 返回最小化的函数，即当没有任何障碍冲突时的路径长度。
    vector<double> calc_path_length(vector<vector<double>> agent_pos, const Args &args);

    // 觅食行为
    tuple<vector<vector<double>>, vector<double>>
    forage(const vector<vector<double>> &agent_pos, const vector<double> &agent_cost, const Args &args, int Visual,
           int trynum);

    // 聚群行为
    tuple<vector<vector<double>>, vector<double>>
    huddle(vector<vector<double>> agent_pos, const Args &args, const vector<vector<double>> &forage_agent_pos,
           vector<double> forage_cost);

    // 追尾行为
    tuple<vector<vector<double>>, vector<double>>
    follow(const vector<vector<double>> &agent_pos, const Args &args, vector<vector<double>> huddle_agent_pos,
           const vector<double> &huddle_cost, const vector<double> &f, int Visual = 200);

    // 随机移动
    vector<vector<double>> moveRandomly(const vector<vector<double>> &agent_pos, double Visual);

    // 差分数组
    vector<vector<double>> diff(const vector<vector<double>> &origin);

    // 打印代价
    static void printCost(const vector<double> &cost);


public:

    // 无参构造
    PSOAFSA();

    // 有参构造：随机数种子
    PSOAFSA(uint32_t seed);

    // PSO-AFSA算法
    tuple<vector<double>, tuple<double, int, int>>
    PSO_AFSA(vector<int> LB, vector<int> UB, int n_pop, int epochs, double phi, double vel_fact, double rad,
             const Args &args, const string &conf_type, string interpolation_mode);

    int getVisual() const;

    void setVisual(int visual);
};


#endif //MYPATHPLANNING_PSOAFSA_H
