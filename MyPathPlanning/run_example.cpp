#include "run_example.h"
#include <chrono>

tuple<vector<vector<double>>, vector<Result>> run_example() {
    cout << "-------------------------- 程序开始 --------------------------" << endl;
    cout << "断点个数：" << nPts << endl;
    cout << "路径插值点个数：" << pointNums << endl;
    cout << "粒子个数：" << nPop << endl;
    cout << "每一次运行迭代次数：" << epochs << endl;
    cout << "插值方法：" << f_interp << endl;
    cout << "--------------------------- GO ON ---------------------------" << endl;

    // start行 goals列 但使用匈牙利算法，要求是n行n列的矩阵
    vector<vector<double>> uav_sum_length(uavs_start.size(),
                                          vector<double>(goals["point_goals"].size() + goals["surface_goals"].size(),
                                                         0));
    // 存储结果
    vector<Result> Map_res;
    // 障碍信息是通用的，获取一次就行，避免重复获取
    vector<Obstacle> obs = get_all_obs();
    // 空间限制
    PathPlanning pathPlanning(limits);
    // 设置全局随机数种子
    uint32_t seed = chrono::system_clock::now().time_since_epoch().count();
    cout << "seed = " << seed << endl;
//    pathPlanning.setSeed(seed);
    pathPlanning.setSeed(1284706116);
    int kv = 80;
    pathPlanning.setVisual(12);

    // 加载障碍信息
    for (const auto &ob: obs) {
        if (ob.type_name == "cir") {
            pathPlanning.add_circle_obs(ob.x, ob.y, ob.real_r, kv);
        } else if (ob.type_name == "con") {
            pathPlanning.add_convex(ob.x, ob.y, ob.V, kv);
        }
    }

    // 存储当前结果信息
    vector<double> best_pos;    // 断点坐标
    double X;                   // 总路径长度
    int count;                  // 路径中有几个点在障碍物内
    vector<double> Px, Py;      // 插值的x坐标 插值的y坐标

    auto start_time = chrono::high_resolution_clock::now();
    // 对每一架无人机进行处理
    for (int uav = 0; uav < uavs_start.size(); uav++) {
        cout << endl;
        cout << "**************************** 第" << uav + 1 << "架飞机 ****************************" << endl;

        int co = 0;
        // 每一架无人机都需要对每一个目标进行处理
        for (const auto &[goal_type, goal_data]: goals) {
            if (goal_type == "point_goals") {
                for (int num = 0; num < goal_data.size(); num++) {
                    cout << "第" << uav + 1 << "架飞机执行" << goal_type << "类型的第" << num + 1 << "个目标" << endl;
                    /*
                     * 设置参数的操作拆解了原本python代码的逻辑，python代码中的逻辑显得冗余
                     * 不过此处性质不变，通过使用setter赋值也不影响效果
                     * （1）空间限制不变，放在循坏外
                     * （2）原本python代码中的start和goal其实赋值的内容是一样的
                     * */
                    // 设置参数 start: 无人机其实位置 goal: 目标的位置
                    pathPlanning.setStart(uavs_start[uav]);
                    pathPlanning.setGoal(goal_data[num]);
                    pathPlanning.setTarget(goal_data[num]);

                    // ---------------------------------开始------------------------------
                    tie(best_pos, X, count, Px, Py) = pathPlanning.optimize(nPts, pointNums, nPop, epochs, 2.05, 0.5, 0.1);

                    PathPlanning _result = pathPlanning;
                    Map_res.emplace_back(best_pos, X, count, Px, Py);

                    double L = _result.getRes().X;
                    int obs_count = _result.getRes().obs_count;

                    cout << "start = [" << pathPlanning.getStart()[0] << ", " << pathPlanning.getStart()[1] << "]";
                    cout << "\tgoal = [" << pathPlanning.getGoal()[0] << ", " << pathPlanning.getGoal()[1] << endl;

                    cout << "路径长度为：" << L << "\tcount = " << obs_count << endl;
                    cout << endl;

                    double l1 = L;
                    double l2 = 0;
                    double l3 = get_l3(goal_data[num], L);
                    uav_sum_length[uav][co++] = l1 + l2 + l3;
                }
            } else if (goal_type == "surface_goals") {
                // 参数声明
                int cycle_count;    // 目标需要的往复覆盖次数
                double goal_length; // 目标的长度（用于计算L2长度）
                for (int num = 0; num < goal_data.size(); num++) {
                    // 第 m 架飞机对第 n 个目标进行规划
                    cout << "第" << uav + 1 << "架飞机执行" << goal_type << "类型的第" << num + 1 << "个目标" << endl;
                    // 计算目标的长度以及需要往复的次数
                    tie(cycle_count, goal_length) = calcu_cycle_count(F, uavs_start[uav][2], goal_data[num]);
                    // 往复次数是1，则为：线型目标
                    if (cycle_count == 1) {
                        cout << "线型目标：" << endl;
                        /*
                         * 设置参数的操作拆解了原本python代码的逻辑，python代码中的逻辑显得冗余
                         * 不过此处性质不变，通过使用setter赋值也不影响效果
                         * （1）空间限制不变，放在循坏外
                         * （2）原本python代码中的start和goal其实赋值的内容是一样的
                         * */
                        // 设置参数 start: 无人机其实位置 goal: 目标的位置
                        // 设置参数
                        pathPlanning.setStart(uavs_start[uav]);
                        pathPlanning.setGoal(goal_data[num]);
                        pathPlanning.setTarget(goal_data[num]);

                        // ---------------------------------开始------------------------------

                        tie(best_pos, X, count, Px, Py) = pathPlanning.optimize(nPts, pointNums, nPop, epochs, 2.05, 0.5, 0.1);
                        PathPlanning _result = pathPlanning;
                        Map_res.emplace_back(best_pos, X, count, Px, Py);

                        double L = _result.getRes().X;
                        int obs_count = _result.getRes().obs_count;

                        cout << "start = [" << pathPlanning.getStart()[0] << ", " << pathPlanning.getStart()[1] << "]";
                        cout << "\tgoal = [" << pathPlanning.getGoal()[0] << ", " << pathPlanning.getGoal()[1] << "]"
                             << endl;

                        cout << "路径长度为：L = " << L << "\t路径中有几个点在障碍物内：count = " << obs_count << endl;
                        cout << endl;

                        double l1 = L;
                        double l2 = get_l2(cycle_count, goal_length);
                        double l3 = get_l3(goal_data[num], l1, cycle_count, goal_length);
                        uav_sum_length[uav][co++] = L + l2 + l3;

                    } else { // 面型目标
                        cout << "面型目标" << endl;
                        // 设置参数
                        pathPlanning.setStart(uavs_start[uav]);
                        pathPlanning.setGoal(goal_data[num]);
                        pathPlanning.setTarget(goal_data[num]);

                        tie(best_pos, X, count, Px, Py) = pathPlanning.optimize(nPts, pointNums, nPop, epochs, 2.05, 0.5, 0.1);
                        PathPlanning _result = pathPlanning;
                        Map_res.emplace_back(best_pos, X, count, Px, Py);

                        double L = _result.getRes().X;
                        int obs_count = _result.getRes().obs_count;

                        cout << "start = [" << pathPlanning.getStart()[0] << ", " << pathPlanning.getStart()[1] << "]";
                        cout << "\tgoal = [" << pathPlanning.getGoal()[0] << ", " << pathPlanning.getGoal()[1] << "]"
                             << endl;

                        cout << "路径长度为：" << L << "\tcount = " << obs_count << endl;
                        cout << endl;

                        double l1 = L;
                        double l2 = get_l2(cycle_count, goal_length);
                        double l3 = get_l3(goal_data[num], l1, cycle_count, goal_length);
                        uav_sum_length[uav][co++] = L + l2 + l3;
                    }
                }
            }
        }
    }
    cout << "--------------------------程序结束--------------------------" << endl;
    auto end_time = chrono::high_resolution_clock::now();
    chrono::duration<double> run_time = end_time - start_time;
    cout << "PSO迭代时间" << run_time.count() << "秒" << endl;

    // 打印代价矩阵
    for (int i = 0; i < uav_sum_length.size(); i++) {
        for (int j = 0; j < uav_sum_length[i].size(); j++) {
            if (j == 0) cout << uav_sum_length[i][j];
            else cout << ", " << uav_sum_length[i][j];
        }
        cout << endl;
    }
    return {uav_sum_length, Map_res};
}