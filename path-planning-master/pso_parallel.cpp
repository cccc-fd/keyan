//
// Created by User on 2025/4/8.
//
#include "pso.h"
#include <omp.h>

/**
 * 并行版本的path_length函数
 */
tuple<vector<double>, vector<vector<double>>, vector<double>>
path_length_parallel(const vector<vector<double>> &positions, const Args &args) {
    int n_pop = positions.size();
    int dim = positions[0].size();

    vector<double> costs(n_pop);
    vector<vector<double>> best_positions(n_pop, vector<double>(dim));
    vector<double> best_costs(n_pop);

    // 将原始的path_length函数拆分为并行计算
#pragma omp parallel for
    for (int i = 0; i < n_pop; i++) {
        // 为每个粒子单独计算适应度
        vector<vector<double>> single_particle_pos;
        single_particle_pos.push_back(positions[i]);

        // 调用原始的path_length函数处理单个粒子
        vector<double> single_costs;
        vector<vector<double>> single_best_pos;
        vector<double> single_best_costs;

        tie(single_costs, single_best_pos, single_best_costs) = path_length(single_particle_pos, args);

        // 保存计算结果
        costs[i] = single_costs[0];
        best_positions[i] = single_best_pos[0];
        best_costs[i] = single_best_costs[0];
    }

    return {costs, best_positions, best_costs};
}

/**
 * 并行版本的随机回退约束
 */


/**
 * 并行版本的双曲线约束
 */
vector<vector<double>> hyperbolic_conf_parallel(const vector<vector<double>> &pos,
                                                const vector<vector<double>> &vel,
                                                const vector<int> &UB, const vector<int> &LB) {
    int n_pop = pos.size();
    int dim = pos[0].size();
    vector<vector<double>> vel_conf(n_pop, vector<double>(dim));

#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            // 双曲线约束逻辑
            double d = 0;
            if (pos[i][j] > UB[j]) {
                d = (pos[i][j] - UB[j]) / (UB[j] - LB[j]);
            } else if (pos[i][j] < LB[j]) {
                d = (LB[j] - pos[i][j]) / (UB[j] - LB[j]);
            }

            vel_conf[i][j] = -vel[i][j] * tanh(d);
        }
    }

    return vel_conf;
}

/**
 * 并行版本的混合约束
 */