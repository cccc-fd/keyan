#ifndef MYPATHPLANNING_RESULT_H
#define MYPATHPLANNING_RESULT_H

#include <iostream>
#include <vector>

#include "obstacle.h"

struct Result {
    // 断点坐标
    std::vector<double> best_pos;
    // 总路径长度
    double X;
    // 路径中有几个障碍点
    int obs_count;
    // 插值的x坐标
    std::vector<double> Px;
    // 插值的y坐标
    std::vector<double> Py;
    // 构造器
    Result(){}
    Result(const std::vector<double> &best_pos, double X, int obs_count, const std::vector<double> &px,
           const std::vector<double> &py) : best_pos(best_pos),
                                            X(X), obs_count(obs_count),
                                            Px(px),
                                            Py(py) {}

};

#endif //MYPATHPLANNING_RESULT_H
