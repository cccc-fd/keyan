#ifndef MYPATHPLANNING_PSO_H
#define MYPATHPLANNING_PSO_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <random>
#include <algorithm>
#include <limits>
#include <omp.h>
#include "args.h"
#include "utils.h"

using namespace std;
// 添加到 pso.h 文件中的适当位置

// 并行版本的适应度计算函数
// 在 pso.h 文件中添加或修改以下函数声明

// 并行版本的适应度计算函数
tuple<vector<double>, vector<vector<double>>, vector<double>>
path_length_parallel(const vector<vector<double>> &positions, const Args &args);

// 并行版本的约束策略函数，注意参数类型是非常量引用
vector<vector<double>> random_back_conf_parallel(const vector<vector<double>> &vel,
                                                 vector<mt19937> &generators);

vector<vector<double>> hyperbolic_conf_parallel(const vector<vector<double>> &pos,
                                                const vector<vector<double>> &vel,
                                                const vector<int> &UB, const vector<int> &LB);

vector<vector<double>> mixed_conf_parallel(const vector<vector<double>> &pos,
                                           const vector<vector<double>> &vel,
                                           const vector<int> &UB, const vector<int> &LB,
                                           vector<mt19937> &generators);
tuple<vector<double>, tuple<double, int, int>>

PSO(vector<int> LB, vector<int> UB, int n_pop, int epochs, int k, double phi, double vel_fact, const string &conf_type,
    const string &intVar, bool normalize, double rad, const Args &args, vector<vector<int>>& xinit);
#endif //MYPATHPLANNING_PSO_H
