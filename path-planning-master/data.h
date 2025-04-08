#ifndef MYPATHPLANNING_DATA_H
#define MYPATHPLANNING_DATA_H

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <map>

/*
 * 一些系统数据
 ************************************* tools begin *******************************************
 * */

extern const double eps;      //1的负6次方


// 无人机起点：x y 高度
extern const int start[3];
// 侦察目标位置（单目标）：x y 高度
extern const int goal[3];
// 空间限制：左、右、上、下、高度
extern const std::vector<int> limits;

/*
 * 多无人集群
 * */
extern const std::vector<std::vector<int>> uavs_start;

// 侦察角度
extern const double F;

extern std::unordered_map<std::string, std::vector<std::vector<int>>> goals;


/**
 * 以下为配置运行信息
 * 此部分信息在分文件的时候口语抽取到一个conf.h文件，使用#define方式定义或许更合适
 *
 */
extern const int nRun;              // 运行次数
extern const int nPts;              // 断点个数
extern const int d;                 // 计算插值点数量因子
extern const int nPop;              // 粒子数量
extern const int epochs;            // 迭代次数
extern const std::string f_interp;       // 插值方法  slinear  quadratic  cubic
extern std::vector<std::vector<int>> Xinit;   // 粒子初始值
extern const int ns;                // 插值点数量

// 所有的运行结果
extern std::vector<std::vector<int> *> result;

// 所有障碍
extern std::unordered_map<std::string, std::vector<std::vector<int>>> obstacle;


#endif //MYPATHPLANNING_DATA_H
