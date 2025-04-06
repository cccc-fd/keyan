#include "pso.h"
#include <cmath>

/**
 * PSO算法
 * @param LB            空间下限
 * @param UB            空间上限
 * @param n_pop         粒子数量
 * @param epochs        迭代次数
 * @param k             置0，整个群体
 * @param phi           计算置信系数
 * @param vel_fact      限制速度，最大和最小
 * @param conf_type     约束类型
 * @param intVar       指定将哪个变量视为整数的索引列表
 * @param normalize     指定是否应该对搜索空间进行规范化（以提高收敛性）
 * @param rad           判断粒子是否符合条件
 * @param args          参数
 * @param xinit         粒子的初始值
 */
tuple<vector<double>, tuple<double, int, int>>
PSO(vector<int> LB, vector<int> UB, int n_pop, int epochs, double phi, double vel_fact, double rad, const Args &args,
    const string &conf_type, string interpolation_mode) {
    // 空间维度 Spatial dimension
    int dim = (int) LB.size();

    // 速度限制
    vector<double> vel_max(dim, 0.0);
    vector<double> vel_min(dim, 0.0);
    for (int i = 0; i < dim; i++) {
        vel_max[i] = vel_fact * (UB[i] - LB[i]);
        vel_min[i] = -vel_max[i];
    }

    // 随机设备
    random_device rd;
    // 随机数生成器
    mt19937 gen(rd());
    // 0-1 之间的随机数
    uniform_real_distribution<> dis(0.0, 1.0);

    //******************************* PSO 算法 **********************************
    // 步骤1：初始化整个种群，尽量将粒子随机的分布在整个搜索空间上，以提高找到最优值的可能性
    // 步骤1.1：定义每个粒子的初始位置，使粒子尽量随机的分布在整个搜索空间
    // 粒子数量：nPop，粒子的维度：nVar；空间限制：[0, 200]
    vector<vector<double>> agent_pos(n_pop, vector<double>(dim));
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            agent_pos[i][j] = LB[j] + dis(gen) * (UB[j] - LB[j]);
        }
    }

    // 步骤1.2：初始化每一个粒子的速度，初始化每个粒子的速度，也是随机的
    // 粒子数量：nPop，粒子维度：dim
    vector<vector<double>> agent_vel(n_pop, vector<double>(dim));
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            // agent_vel[i][j] = (LB[j] - agent_pos[i][j]) + (double) rand() / RAND_MAX * (UB[j] - LB[j]);
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

    // 计算代价函数 得到代价
    tie(agent_cost, afas_pos, afas_cost) = path_length(agent_pos, args);

    // 找出此次迭代结果的最优解
    vector<bool> better;
    better.reserve(n_pop);
    for (int i = 0; i < n_pop; i++) {
        better.push_back(afas_cost[i] < agent_cost[i]);
    }
    for (int i = 0; i < n_pop; i++) {
        if (better[i]) {
            // 本次迭代中找到的最好位置
            agent_pos[i].assign(afas_pos[i].begin(), afas_pos[i].end());
            // 本次最好位置的代价
            agent_cost[i] = afas_cost[i];
        }
    }

    // 更新每个粒子的最好pos和cost
    vector<vector<double>> agent_best_pos(agent_pos);
    vector<double> agent_best_cost(agent_cost);

    // 更新全局的最好pos和cost
    // 行代表粒子的个数
    // 找到最好的粒子位置
    int idx = min_element(agent_best_cost.begin(), agent_best_cost.end()) - agent_best_cost.begin();
    vector<double> swarm_best_pos(agent_best_pos[idx].begin(), agent_best_pos[idx].end());
    double swarm_best_cost = agent_best_cost[idx];
    int swarm_best_idx = idx;

    // 全局最优 使用 swarm_best_pos 填充
    vector<vector<double>> group_best_pos(n_pop);
    group_best_pos.assign(n_pop, swarm_best_pos);

    vector<double> p_equal_g(n_pop, 1.0);
    p_equal_g[idx] = 0.75;

    // per_cost: 记录所有最小代价，可以用于看代价是如何下降的
    vector<double> per_cost;
    per_cost.reserve(n_pop);

    // 记录当前最小代价 有些抽象，为什么会 * 2 + 20
    per_cost.push_back(agent_best_cost[idx] * 2 + 20);

    //************************************** 开始迭代 *****************************************

    for (int epoch = 0; epoch < epochs; epoch++) {
        // 动态更新惯性因子 w
        double w = 0.9 - epoch * (0.9 - 0.2) / epochs;
        // 步骤3：更新粒子的状态
        // 行代表第i个粒子 列代表维度
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                // 步骤3.1：更新粒子的速度参数是  c1 = 1.5  c2 = 1.2
                agent_vel[i][j] = w * agent_vel[i][j] + 1.5 * (agent_best_pos[i][j] - agent_pos[i][j]) +
                                  1.2 * (group_best_pos[i][j] - agent_pos[i][j]);
                // 速度限制，速度不能太大，也不能太小
                agent_vel[i][j] = fmin(fmax(agent_vel[i][j], vel_min[j]), vel_max[j]);
            }
        }

        vector<vector<double>> agent_pos_tmp(n_pop, vector<double>(dim));

        // 计算速度更新后的位置
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                agent_pos_tmp[i][j] = agent_pos[i][j] + agent_vel[i][j];
            }
        }

        // 标识粒子是否超出范围
        vector<vector<bool>> out(n_pop, vector<bool>(dim, false));
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                if (agent_pos_tmp[i][j] > LB[j] || agent_pos_tmp[i][j] < UB[j]) {
                    out[i][j] = true;
                }
            }
        }

        // 对速度进行限制 改变速度 使得粒子加上速度后不超过范围
        vector<vector<double>> vel_conf;
        if (conf_type == "RB") {
            vel_conf = random_back_conf(agent_vel);
        } else if (conf_type == "HY") {
            vel_conf = hyperbolic_conf(agent_pos, agent_vel, UB, LB);
        } else if (conf_type == "MX") {
            vel_conf = mixed_conf(agent_pos, agent_vel, UB, LB);
        }

        // 对出界粒子应用限制规则
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                if (out[i][j]) {
                    agent_vel[i][j] = vel_conf[i][j];
                }
                // 对速度进行更新后，再更新位置
                agent_pos[i][j] += agent_vel[i][j];
            }
        }

        // 对依然出界的粒子进行限制边界
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                agent_pos[i][j] = fmin(fmax(agent_pos[i][j], LB[j]), UB[j]);
            }
        }

        // 计算更新后的损失
        tie(agent_cost, afas_pos, afas_cost) = path_length(agent_pos, args);

        // 找出此次迭代结果的最优解
        for (int i = 0; i < n_pop; i++) {
            if (afas_cost[i] < agent_cost[i]) {
                agent_pos[i].assign(afas_pos[i].begin(), afas_pos[i].end());
                agent_cost[i] = afas_cost[i];
            }
        }

        // 更新个体历史最优
        for (int i = 0; i < agent_best_pos.size(); i++) {
            if (agent_cost[i] < agent_best_cost[i]) {
                agent_best_pos[i].assign(agent_pos[i].begin(), agent_pos[i].end());
                agent_best_cost[i] = agent_cost[i];
            }
        }

        // 更新全局历史最优
        idx = min_element(agent_best_cost.begin(), agent_best_cost.end()) - agent_best_cost.begin();
        if (agent_best_cost[idx] < swarm_best_cost) {
            swarm_best_pos.assign(agent_best_pos[idx].begin(), agent_best_pos[idx].end());
            swarm_best_cost = agent_best_cost[idx];
            swarm_best_idx = idx;
        }

        // 全局极值
        group_best_pos.assign(n_pop, swarm_best_pos);
        p_equal_g.assign(n_pop, 1.0);
        p_equal_g[idx] = 0.75;
        per_cost.push_back(agent_best_cost[idx] * 2 + 20);

    }
    //<-------------------------------------- 迭代完毕 --------------------------------------------->

    vector<vector<double>> delta(n_pop, vector<double>(dim));
    vector<double> deltaB(dim);

    for (int i = 0; i < dim; i++) {
        deltaB[i] = fmax(UB[i] - LB[i], 1e-10);
    }
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            delta[i][j] = (agent_best_pos[i][j] - swarm_best_pos[j]) / deltaB[j];
        }
    }

    // 为计算范数做准备
    vector<vector<double>> tmp1(n_pop, vector<double>(dim));
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            tmp1[i][j] = delta[i][j] / sqrt(n_pop);
        }
    }
    vector<double> dist(n_pop);
    for (int i = 0; i < n_pop; i++) {
        double _sum = 0;
        // 每一行的平方和
        for (int j = 0; j < dim; j++) {
            _sum += tmp1[i][j] * tmp1[i][j];
        }
        dist[i] = sqrt(_sum);
    }

    int in_rad = 0;
    for (double e: dist) {
        if (e < rad) in_rad++;
    }
    // 最优cost，最优粒子索引，小于rad的粒子数
    tuple<double, int, int> infor = {swarm_best_cost, swarm_best_idx, in_rad};
    printCost(per_cost);

    return {swarm_best_pos, infor};
}