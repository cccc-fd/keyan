#include "pso.h"
#include <cmath>
#include <omp.h>

/**
 * PSO算法
 * [原有函数描述保持不变]
 */
tuple<vector<double>, tuple<double, int, int>>
PSO(vector<int> LB, vector<int> UB, int n_pop, int epochs, int k, double phi, double vel_fact, const string &conf_type,
    const string &intVar, bool normalize, double rad, const Args &args, vector<vector<int>> &xinit) {
    // 设置OpenMP线程数，可以根据系统情况调整
    int num_threads = omp_get_max_threads();
    omp_set_num_threads(num_threads);

    // 空间维度 Spatial dimension
    int dim = (int) LB.size();

    // 速度限制
    vector<double> vel_max(dim, 0.0);
    vector<double> vel_min(dim, 0.0);
#pragma omp parallel for
    for (int i = 0; i < dim; i++) {
        vel_max[i] = vel_fact * (UB[i] - LB[i]);
        vel_min[i] = -vel_max[i];
    }

    // 每个线程的随机数生成器
    vector<mt19937> generators(num_threads);
#pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        unsigned int seed = thread_id + static_cast<unsigned int>(time(nullptr));
        generators[thread_id] = mt19937(seed);
    }

    //******************************* PSO 算法 **********************************
    // 步骤1：初始化整个种群，尽量将粒子随机的分布在整个搜索空间上，以提高找到最优值的可能性
    // 步骤1.1：定义每个粒子的初始位置，使粒子尽量随机的分布在整个搜索空间
    vector<vector<double>> agent_pos(n_pop, vector<double>(dim));
#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            int thread_id = omp_get_thread_num();
            uniform_real_distribution<> dis(0.0, 1.0);
            mt19937& gen = generators[thread_id]; // 获取非常量引用
            agent_pos[i][j] = LB[j] + dis(gen) * (UB[j] - LB[j]);
        }
    }

    // 步骤1.2：初始化每一个粒子的速度，初始化每个粒子的速度，也是随机的
    vector<vector<double>> agent_vel(n_pop, vector<double>(dim));
#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            int thread_id = omp_get_thread_num();
            uniform_real_distribution<> dis(0.0, 1.0);
            mt19937& gen = generators[thread_id]; // 获取非常量引用
            agent_vel[i][j] = (LB[j] - agent_pos[i][j]) + dis(gen) * (UB[j] - LB[j]);
            // 限制速度不能大于vel_max 不能小于vel_min
            agent_vel[i][j] = fmin(fmax(agent_vel[i][j], vel_min[j]), vel_max[j]);
        }
    }

    // 步骤2：计算个体适应度
    vector<vector<double>> agent_pos_orig(n_pop, vector<double>(dim));
    vector<double> agent_cost(n_pop);
    vector<vector<double>> afas_pos(n_pop, vector<double>(dim));
    vector<double> afas_cost(n_pop);

    // 并行计算适应度函数
    tie(agent_cost, afas_pos, afas_cost) = path_length_parallel(agent_pos, args);

    // 找出此次迭代结果的最优解
    vector<bool> better(n_pop);
#pragma omp parallel for
    for (int i = 0; i < n_pop; i++) {
        better[i] = afas_cost[i] < agent_cost[i];
    }

#pragma omp parallel for
    for (int i = 0; i < n_pop; i++) {
        if (better[i]) {
            // 本次迭代中找到的最好位置
            agent_pos[i].assign(afas_pos[i].begin(), afas_pos[i].end());
            // 本次最好位置的代价
            agent_cost[i] = afas_cost[i];
        }
    }

    // 更新每个粒子的最好pos和cost
    vector<vector<double>> agent_best_pos = agent_pos;
    vector<double> agent_best_cost = agent_cost;

    // 更新全局的最好pos和cost - 使用并行归约找最小值
    int idx = 0;
    double min_cost = agent_best_cost[0];
#pragma omp parallel
    {
        int local_idx = 0;
        double local_min = agent_best_cost[0];

#pragma omp for nowait
        for (int i = 1; i < n_pop; i++) {
            if (agent_best_cost[i] < local_min) {
                local_min = agent_best_cost[i];
                local_idx = i;
            }
        }

#pragma omp critical
        {
            if (local_min < min_cost) {
                min_cost = local_min;
                idx = local_idx;
            }
        }
    }

    vector<double> swarm_best_pos = agent_best_pos[idx];
    double swarm_best_cost = agent_best_cost[idx];
    int swarm_best_idx = idx;

    // 全局最优 使用 swarm_best_pos 填充
    vector<vector<double>> group_best_pos(n_pop);
#pragma omp parallel for
    for (int i = 0; i < n_pop; i++) {
        group_best_pos[i] = swarm_best_pos;
    }

    vector<double> p_equal_g(n_pop, 1.0);
    p_equal_g[idx] = 0.75;

    // per_cost: 记录所有最小代价，可以用于看代价是如何下降的
    vector<double> per_cost;
    per_cost.reserve(epochs + 1);
    per_cost.push_back(agent_best_cost[idx] * 2 + 20);

    //************************************** 开始迭代 *****************************************
    for (int epoch = 0; epoch < epochs; epoch++) {
        // 动态更新惯性因子 w
        double w = 0.9 - epoch * (0.9 - 0.2) / epochs;

        // 步骤3：更新粒子的状态 - 并行更新粒子速度
#pragma omp parallel for collapse(2)
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                // 步骤3.1：更新粒子的速度
                agent_vel[i][j] = w * agent_vel[i][j] + 1.5 * (agent_best_pos[i][j] - agent_pos[i][j]) +
                                  1.2 * (group_best_pos[i][j] - agent_pos[i][j]);
                // 速度限制
                agent_vel[i][j] = fmin(fmax(agent_vel[i][j], vel_min[j]), vel_max[j]);
            }
        }

        // 并行计算临时位置
        vector<vector<double>> agent_pos_tmp(n_pop, vector<double>(dim));
#pragma omp parallel for collapse(2)
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                agent_pos_tmp[i][j] = agent_pos[i][j] + agent_vel[i][j];
            }
        }

        // 并行检查粒子是否超出范围
        vector<vector<bool>> out(n_pop, vector<bool>(dim, false));
#pragma omp parallel for collapse(2)
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                if (agent_pos_tmp[i][j] > UB[j] || agent_pos_tmp[i][j] < LB[j]) {
                    out[i][j] = true;
                }
            }
        }

        // 对速度进行限制 - 使用并行版本的约束函数
        vector<vector<double>> vel_conf;
        if (conf_type == "RB") {
            vel_conf = random_back_conf_parallel(agent_vel, generators);
        } else if (conf_type == "HY") {
            vel_conf = hyperbolic_conf_parallel(agent_pos, agent_vel, UB, LB);
        } else if (conf_type == "MX") {
            vel_conf = mixed_conf_parallel(agent_pos, agent_vel, UB, LB, generators);
        }

        // 并行应用限制并更新位置
#pragma omp parallel for collapse(2)
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                if (out[i][j]) {
                    agent_vel[i][j] = vel_conf[i][j];
                }
                // 更新位置
                agent_pos[i][j] += agent_vel[i][j];
            }
        }

        // 并行处理边界限制
#pragma omp parallel for collapse(2)
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                agent_pos[i][j] = fmin(fmax(agent_pos[i][j], LB[j]), UB[j]);
            }
        }

        // 并行计算适应度
        tie(agent_cost, afas_pos, afas_cost) = path_length_parallel(agent_pos, args);

        // 并行更新最优位置
#pragma omp parallel for
        for (int i = 0; i < n_pop; i++) {
            if (afas_cost[i] < agent_cost[i]) {
                agent_pos[i].assign(afas_pos[i].begin(), afas_pos[i].end());
                agent_cost[i] = afas_cost[i];
            }
        }

        // 并行更新个体历史最优
#pragma omp parallel for
        for (int i = 0; i < agent_best_pos.size(); i++) {
            if (agent_cost[i] < agent_best_cost[i]) {
                agent_best_pos[i].assign(agent_pos[i].begin(), agent_pos[i].end());
                agent_best_cost[i] = agent_cost[i];
            }
        }

        // 并行查找全局最优
        idx = 0;
        min_cost = agent_best_cost[0];
#pragma omp parallel
        {
            int local_idx = 0;
            double local_min = agent_best_cost[0];

#pragma omp for nowait
            for (int i = 1; i < n_pop; i++) {
                if (agent_best_cost[i] < local_min) {
                    local_min = agent_best_cost[i];
                    local_idx = i;
                }
            }

#pragma omp critical
            {
                if (local_min < min_cost) {
                    min_cost = local_min;
                    idx = local_idx;
                }
            }
        }

        // 更新全局历史最优
        if (agent_best_cost[idx] < swarm_best_cost) {
            swarm_best_pos.assign(agent_best_pos[idx].begin(), agent_best_pos[idx].end());
            swarm_best_cost = agent_best_cost[idx];
            swarm_best_idx = idx;
        }

        // 更新全局最优到所有粒子
#pragma omp parallel for
        for (int i = 0; i < n_pop; i++) {
            group_best_pos[i] = swarm_best_pos;
        }

        p_equal_g.assign(n_pop, 1.0);
        p_equal_g[idx] = 0.75;
        per_cost.push_back(agent_best_cost[idx] * 2 + 20);
    }
    //<-------------------------------------- 迭代完毕 --------------------------------------------->

    // 并行计算多样性指标
    vector<vector<double>> delta(n_pop, vector<double>(dim));
    vector<double> deltaB(dim);

#pragma omp parallel for
    for (int i = 0; i < dim; i++) {
        deltaB[i] = fmax(UB[i] - LB[i], 1e-10);
    }

#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            delta[i][j] = (agent_best_pos[i][j] - swarm_best_pos[j]) / deltaB[j];
        }
    }

    // 并行准备范数计算
    vector<vector<double>> tmp1(n_pop, vector<double>(dim));
#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            tmp1[i][j] = delta[i][j] / sqrt(n_pop);
        }
    }

    // 并行计算距离
    vector<double> dist(n_pop);
#pragma omp parallel for
    for (int i = 0; i < n_pop; i++) {
        double _sum = 0;
        for (int j = 0; j < dim; j++) {
            _sum += tmp1[i][j] * tmp1[i][j];
        }
        dist[i] = sqrt(_sum);
    }

    // 并行计算在rad范围内的粒子数
    int in_rad = 0;
#pragma omp parallel
    {
        int local_count = 0;
#pragma omp for nowait
        for (int i = 0; i < n_pop; i++) {
            if (dist[i] < rad) local_count++;
        }

#pragma omp critical
        {
            in_rad += local_count;
        }
    }

    // 返回结果
    tuple<double, int, int> infor = {swarm_best_cost, swarm_best_idx, in_rad};
    printCost(per_cost);

    return {swarm_best_pos, infor};
}

/**
 * 并行版本的随机回退约束
 */
vector<vector<double>> random_back_conf_parallel(const vector<vector<double>> &vel,
                                                 vector<mt19937> &generators) {
    int n_pop = vel.size();
    int dim = vel[0].size();
    vector<vector<double>> vel_conf(n_pop, vector<double>(dim));

#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            int thread_id = omp_get_thread_num();
            uniform_real_distribution<> dis(0.0, 1.0);
            mt19937& gen = generators[thread_id];  // 获取非常量引用
            vel_conf[i][j] = -vel[i][j] * dis(gen);  // 实际使用随机数
        }
    }
    return vel_conf;
}

/**
 * 并行版本的混合约束
 */
vector<vector<double>> mixed_conf_parallel(const vector<vector<double>> &pos,
                                           const vector<vector<double>> &vel,
                                           const vector<int> &UB, const vector<int> &LB,
                                           vector<mt19937> &generators) {  // 非常量引用
    int n_pop = pos.size();
    int dim = pos[0].size();
    vector<vector<double>> vel_conf(n_pop, vector<double>(dim));

#pragma omp parallel for collapse(2)
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            int thread_id = omp_get_thread_num();
            uniform_real_distribution<> dis(0.0, 1.0);
            mt19937& gen = generators[thread_id];  // 获取非常量引用

            // 混合约束策略
            if (dis(gen) < 0.5) {  // 使用非常量引用
                // 使用随机回退约束
                vel_conf[i][j] = -vel[i][j] * dis(gen);  // 使用非常量引用
            } else {
                // 使用双曲线约束
                double d = 0;
                if (pos[i][j] > UB[j]) {
                    d = (pos[i][j] - UB[j]) / (UB[j] - LB[j]);
                } else if (pos[i][j] < LB[j]) {
                    d = (LB[j] - pos[i][j]) / (UB[j] - LB[j]);
                }

                vel_conf[i][j] = -vel[i][j] * tanh(d);
            }
        }
    }

    return vel_conf;
}