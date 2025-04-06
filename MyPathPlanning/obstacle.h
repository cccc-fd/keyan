#ifndef MYPATHPLANNING_OBSTACLE_H
#define MYPATHPLANNING_OBSTACLE_H

#include <string>
#include <vector>

struct Obstacle {
    // 障碍类型：cir or conv
    std::string type_name;
    // 障碍中心点(x, y)
    double x, y;
    // 切面圆半径
    double real_r;
    //
    double kv;
    // 禁飞区二维数组
    std::vector<std::vector<int>> V;
};

#endif //MYPATHPLANNING_OBSTACLE_H
