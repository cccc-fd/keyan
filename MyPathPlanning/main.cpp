#include <iostream>
#include <vector>
#include <map>
#include <chrono>

#include "utils.h"
#include "result.h"
#include "Hungarian.h"
#include "Example.h"
#include "run_example.h"

using namespace std;

int main() {
    // 防止中文乱码
    system("chcp 65001");
    // 获取时间
    auto start_time = chrono::high_resolution_clock::now();

    // 代价矩阵
    vector<vector<double>> uav_sum_length;
    // 路径结点
    vector<Result> Map_res;
     tie(uav_sum_length, Map_res) = run_example();
//    Example example;
//    tie(uav_sum_length, Map_res) = example.run();

    int l = uav_sum_length.size();

    tuple<vector<tuple<int, int>>, double> data;

    Hungarian hungarian;
    auto hungarian_start_time = chrono::high_resolution_clock::now();
    data = hungarian.hungarian(uav_sum_length);


    vector<Result> res_1, res_2, res_3, res_4;
    // 考虑预分配的规划结果
    vector<Result> end_res;
    // 未考虑预分配的规划结果

    int index1 = 0, index2 = 4, index3 = 8, index4 = 12;

    for (int i = 0; i < 4; i++) {
        res_1.push_back(Map_res[i + index1]);
        res_2.push_back(Map_res[i + index2]);
        res_3.push_back(Map_res[i + index3]);
        res_4.push_back(Map_res[i + index4]);
    }

    cout << "******************** 分配方案 ********************" << endl;
    vector<tuple<int, int>> datas;
    double cost_sum;
    tie(datas, cost_sum) = data;

    for (int i = 0; i < datas.size(); i++) {
        auto key = get<0>(datas[i]);
        auto val = get<1>(datas[i]);
        cout << key << " 号无人机探查目标 " << val << endl;
        end_res.push_back(Map_res[(key - 1) * l + val - 1]);
    }
    cout << "总代价为：" << cost_sum << endl;

    auto hungarian_end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> hungarian_run_time = hungarian_end_time - hungarian_start_time;
    cout << "hungarian run time = " << hungarian_run_time.count() << endl;

    // 运行结束时间
    auto end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> run_time = end_time - start_time;
    cout << "运行时间：" << run_time.count() << "秒\n";

    outResult(end_res);

    return 0;
}