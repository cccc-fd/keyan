#ifndef MYPATHPLANNING_ARGS_H
#define MYPATHPLANNING_ARGS_H

#include <vector>
#include <string>
#include <tuple>

#include "obstacle.h"

struct Args {
    // 起始位置（一维）
    std::vector<int> starts_odv;
    // 目标位置（一维）
    std::vector<int> goals_odv;
    // 起始位置（一维）
    std::vector<std::vector<int>> starts_tdv;
    // 目标位置（二维）
    std::vector<std::vector<int>> goals_tdv;
    // 障碍数组
    std::vector<Obstacle> obs;
    // 路径点数量
    int interpolationPointNums;
    // 插值方式（其实这个类型可以省略）
    std::string interpolationKind;
    // 返回结果
    std::tuple<double, int, std::vector<double>, std::vector<double>> res;
    // 参数数量
    int argsSize;

    // 构造器，一维
    Args(const std::vector<int> &starts, const std::vector<int> &goals, const std::vector<Obstacle> &obs,
         const int &interpolationNums, const std::string &type) : starts_odv(starts),
                                                                  goals_odv(goals),
                                                                  obs(obs),
                                                                  interpolationPointNums(
                                                                          interpolationNums),
                                                                  interpolationKind(type) { this->argsSize = 6; }

    // 构造器，二维
    Args(const std::vector<std::vector<int>> &starts, const std::vector<std::vector<int>> &goals,
         std::vector<Obstacle> obs, const int &interNums, const std::string &kind) : starts_tdv(starts),
                                                                                     goals_tdv(goals), obs(obs),
                                                                                     interpolationPointNums(interNums),
                                                                                     interpolationKind(
                                                                                             kind) { this->argsSize = 5; }
};

#endif //MYPATHPLANNING_ARGS_H
