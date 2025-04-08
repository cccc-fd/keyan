#define _USE_MATH_DEFINES
#include "data.h"

const double eps = 1e-6;

// 无人机起点：x y 高度
const int start[3] = {10, 70, 10};
// 侦察目标位置（单目标）：x y 高度
const int goal[3] = {60, 180, 10};
// 空间限制：左、右、上、下、高度
const std::vector<int> limits = {0, 200, 0, 200, 50};

/*
 * 无人机群信息
 * 分别表示：{x, y, h}，即：{x, y, 高度}
 * */
const std::vector<std::vector<int>> uavs_start = {
        {10, 40, 10},
        {20, 20, 10},
        {40, 10, 10},
        {70, 20, 10}
};

/* 侦察角度 π/2
 * 此处 M_PI 是在<cmath.h>中的宏变量，在vs中需要在本文文件声明：#define _USE_MATH_DEFINES
 * */
const double F = M_PI / 2;

/*
 * 目标位置信息
 * key: 目标种类
 * value: 点坐标
 * 其中，value的赋值具有一定的规律
 * 三个数为一个点的坐标，如：{55, 175, 10}是一个点的坐标，代表{x, y, h}，其中h代表高度
 * 四个点的顺序为：西南、东南、东北、西北四个角的顺序
 * */
std::unordered_map<std::string, std::vector<std::vector<int>>> goals = {
        // 点目标
        {"point_goals",   {
//                                  {10, 40,  10},
//                                  {20,  20,  10},
//                                  {40,  10,  10},
//                                  {70,  20, 10}
                          }},
        {"surface_goals", {
                                  {55, 175, 10, 65, 175, 10, 65, 185, 10, 55, 185, 10},
                                  {155, 125, 10, 165, 125, 10, 165, 135, 10, 155, 135, 10},
                                  {130, 170, 10, 190, 170, 10, 190, 180, 10, 130, 180, 10},
                                  {160, 80, 10, 200, 80, 10, 200, 110, 10, 160, 110, 10}
                          }}
};


/**
 * 以下为配置运行信息
 * 此部分信息在分文件的时候口语抽取到一个conf.h文件，使用#define方式定义或许更合适
 */
const int nRun = 15;                    // 运行次数
const int nPts = 3;                     // 断点个数
const int d = 50;                       // 计算插值点数量因子
const int nPop = 100;                   // 粒子数量
const int epochs = 100;                 // 迭代次数
const std::string f_interp = "cubic";   // 插值方法  slinear  quadratic  cubic
std::vector<std::vector<int>> Xinit = {};    // 粒子初始值
const int ns = 1 + (nPts + 1) * d;      // 插值点数量

// 所有的运行结果
std::vector<std::vector<int> *> result(nRun, nullptr);

std::unordered_map<std::string, std::vector<std::vector<int>>> obstacle = {
        {"radar",    {{130, 100, 0,  20}}},
        {"gun",      {{120, 40,  0,  30}, {130, 70,  0,  20}}},
//        {"missile",  {{90,  150, 0,  15}}},
        {"missile",  {{70,  150, 0,  15}}},
        {"mountain", {{50,  40,  15, 10}}},
        {"weather",  {{70,  90,  10, 20}, {50,  105, 10, 20}}},
        {"no_fly",   {{20,  110, 50, 110, 50, 150, 40, 150, 20, 140}}}
};
