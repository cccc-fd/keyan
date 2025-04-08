#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "data.h"
#include "utils.h"
#include "result.h"
#include "Hungarian.h"
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
    tie(uav_sum_length, Map_res) = run_example_2();
   //调用run_example_2实现路径规划
    int l = uav_sum_length.size();

    tuple<vector<tuple<int, int>>, double> data;

    Hungarian hungarian;
    auto hungarian_start_time = chrono::high_resolution_clock::now();
    data = hungarian.hungrian(uav_sum_length);
   //使用匈牙利算法进行优化

    vector<Result> res_1, res_2, res_3, res_4;
    // 考虑预分配的规划结果
    vector<Result> end_res;
    // 未考虑预分配的规划结果
    vector<Result> end_res_no;

    int index1 = 0, index2 = 4, index3 = 8, index4 = 12;

    for (int i = 0; i < 4; i++) {
        res_1.push_back(Map_res[i + index1]);
        res_2.push_back(Map_res[i + index2]);
        res_3.push_back(Map_res[i + index3]);
        res_4.push_back(Map_res[i + index4]);
    }

    cout << "分配方案\t考虑预规划" << endl;
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


    cout << "************ 未考虑预规划分配方案 ************" << endl;

    vector<int> row;

    vector<vector<int>> starts_no;
    for (auto u_start: uavs_start) {
        row.clear();
        row.push_back(u_start[0]);
        row.push_back(u_start[1]);
        starts_no.push_back(row);
    }

    vector<vector<int>> goals_no;
    for (auto [name, _goal]: goals) {
        for (int i = 0; i < _goal.size(); i++) {
            row.clear();
            row.push_back(_goal[i][0]);
            row.push_back(_goal[i][1]);
            goals_no.push_back(row);
        }
    }

    vector<vector<double>> uav_sum_length_no = calcu_line_start_target(starts_no, goals_no);

    double cost_no = 0;
    vector<vector<int>> data_no = {
            {1, 2},
            {2, 1},
            {3, 4},
            {4, 3}
    };
    for (int i = 0; i < data_no.size(); i++) {
        int key = data_no[i][0];
        int val = data_no[i][1];
        cout << key << " 号无人机探查目标 " << val << endl;
        end_res_no.push_back(Map_res[(key - 1) * l + val - 1]);
        cost_no += uav_sum_length[key - 1][val - 1];
    }
    cout << "总代价为：" << cost_no << endl;

    auto hungarian_end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> hungarian_run_time = hungarian_end_time - hungarian_start_time;
    cout << "hungarian run time = " << hungarian_run_time.count() << endl;

    // 运行结束时间
    auto end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> run_time = end_time - start_time;
    cout << "运行时间：" << run_time.count() << "秒\n";

    vector<Obstacle> _obs;
    vector<Obstacle> obs = get_all_obs();

    vector<vector<int>> starts, targets, _goals;
    for (const auto &ob: obs) {
        if (ob.type_name == "cir") {
            _obs.push_back(add_circle_obs(ob.x, ob.y, ob.real_r, ob.kv));
        } else if (ob.type_name == "con") {
            _obs.push_back(add_convex(ob.x, ob.y, ob.V, ob.kv));
        }
    }

    for (auto [name, cur_goal]: goals) {
        for (const auto &item: cur_goal) {
            targets.push_back(item);
        }
    }

    for (const auto &sta: uavs_start) {
        vector<int> tmp;
        tmp.push_back(sta[0]);
        tmp.push_back(sta[1]);
        starts.push_back(tmp);
    }

    for (auto [key, val]: goals) {
        for (const auto &item: val) {
            _goals.push_back(item);
        }
    }

    /*
     * TODO 绘制无人机
     * */

    return 0;
}