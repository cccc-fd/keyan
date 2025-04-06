//
// Created by 15017 on 2024/5/27.
//

#include "Example.h"


// ********************************* public methods begin *****************************

/**
 * 运行案例
 * @return
 */
tuple<vector<vector<double>>, vector<Result>> Example::run() {

    // 查看当前设置的参数信息
    printInformation();

    // start行 goals列 但使用匈牙利算法，要求是n行n列的矩阵
    vector<vector<double>> uav_sum_length(uavs_start.size(),
                                          vector<double>(goals["point_goals"].size() + goals["surface_goals"].size(),
                                                         0));
    // 存储结果
    // vector<Result> Map_res;
    // 障碍信息是通用的，获取一次就行，避免重复获取
    vector<Obstacle> obs = get_all_obs();
    // 空间限制
    // PathPlanning pathPlanning(limits);
    pathPlanning.setLimits(limits);
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


Example::Example() = default;

/**
 * 构造器：初始化运行参数
 * @param breakpointNum
 * @param pointNum
 * @param particleNum
 * @param epochs
 * @param interpType
 */
Example::Example(int breakpointNum, int pointNum, int particleNum, int epochs, const string &interpType) {
    this->breakpointNum = breakpointNum;
    this->pointNum = pointNum;
    this->particleNum = particleNum;
    this->epochs = epochs;
    this->interpType = interpType;
}


// ********************************* public methods end *****************************



// ********************************* private methods begin *****************************

/**
 * 打印当前用例信息
 */
void Example::printInformation() {
    cout << "-------------------------- 程序开始 --------------------------" << endl;
    cout << "断点个数：" << this->breakpointNum << endl;
    cout << "路径插值点个数：" << this->pointNum << endl;
    cout << "粒子个数：" << this->particleNum << endl;
    cout << "每一次运行迭代次数：" << this->epochs << endl;
    cout << "插值方法：" << this->interpType << endl;
}

/**
 * 得到所有的障碍信息，即将障碍坐标转化为具体的切面圆或者其他类型
 * @return 障碍切面数组
 */
vector<Obstacle> Example::get_all_obs() {
    // 障碍切面信息集合
    vector<Obstacle> obs;
    double x, y;
    double real_r;
    // 遍历map，障碍信息
    for (auto [key, val]: obstacle) {
        if (key == "radar" || key == "gun" || key == "missile") {
            for (auto data: val) {
                tie(x, y, real_r) = get_circle(data[0], data[1], data[2], data[3], start[2]);
                // 向obs中追加数据
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "mountain") {
            for (auto data: val) {
                tie(x, y, real_r) = get_mountain_circle(data[0], data[1], data[2], data[3], start[2]);
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "weather") {
            for (auto data: val) {
                tie(x, y, real_r) = get_weather_circle(data[0], data[1], data[2], data[3], start[2]);
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "no_fly") {
            for (const auto &data: val) {
                // 用于存储不规则多边形的各边信息
                vector<vector<int>> V;
                for (int i = 0; i < data.size(); i += 2) {
                    vector<int> v1 = {data[i], data[i + 1]};
                    V.push_back(v1);
                }
                tie(x, y) = centroid(V);
                obs.push_back({"con", x, y, 0.0, 0.0, V});
            }
        }
    }
    return obs;
}


/**
 * 球：获得切面圆半径
 * @param x：x
 * @param y: y
 * @param z: z
 * @param r: 半径
 * @param h: 高度
 * @return 返回值先使用元组接受，后期确定后应当定义结构体或者类来封装数据
 */
tuple<double, double, double> Example::get_circle(int x, int y, int z, int r, int h) {
    double real_r = sqrt(r * r - (h - z) * (h - z));
    return {x, y, real_r};
}

/**
 * 山体：抽象为圆锥
 * @param x
 * @param y
 * @param z
 * @param r
 * @param h
 * @return
 */
tuple<double, double, double> Example::get_mountain_circle(int x, int y, int z, int r, int h) {
    double real_r = r * ((z - (h - z)) / static_cast<double>(z));
    return {x, y, real_r};
}

/**
 * 天气：抽象为圆柱
 * @param x
 * @param y
 * @param z
 * @param r
 * @param h
 * @return
 */
tuple<double, double, double> Example::get_weather_circle(int x, int y, int z, int r, int h) {
    return {x, y, r};
}

/**
 * 多边形
 * @param V 多边形的端点，应当是从右下角开始的，逆时针
 * @return 多边形的中间点
 */
tuple<double, double> Example::centroid(const vector<vector<int>> &V) {
    // 端点个数
    int n_pts = V.size();

    double xc = 0.0;
    double yc = 0.0;
    double A = 0.0;

    for (int i = 0; i < n_pts; ++i) {
        int index = ((i - 1) + n_pts) % n_pts;
        double _d = V[index][0] * V[i][1] - V[i][0] * V[index][1];
        xc += (V[index][0] + V[i][0]) * _d;
        yc += (V[index][1] + V[i][1]) * _d;
        A += _d;
    }

    A = A / 2.0;
    xc = xc / (6.0 * A);
    yc = yc / (6.0 * A);
    return {xc, yc};
}

/**
 * 计算实现全覆盖目标的长度   l2
 * @param cycle_count 往复次数
 * @param goal_length 目标长度
 * @return
 */
double Example::get_l2(const int &cycle_count, const double &goal_length) {
    return cycle_count * goal_length;
}

/**
 * 得到返航的长度l3
 * @param _goal         目标位置（4端点）
 * @param l1            到达目标位置的长度
 * @param cycle_count   往复次数
 * @param goal_length   目标长度
 * @return
 */
double Example::get_l3(vector<int> _goal, double l1, int cycle_count, double goal_length) {
    double l3 = 0.0;
    if (cycle_count == 0) { // 点型目标
        l3 = l1;
    } else if (cycle_count == 1) { // 线型目标
        l3 = goal_length + l1;
    } else if (cycle_count > 1) { // 面型目标
        if (cycle_count % 2 == 0) {
            l3 = l1 + _goal[10] - _goal[1];
        } else {
            l3 = l1 +
                 sqrt((_goal[0] - _goal[6]) * (_goal[0] - _goal[0]) + (_goal[1] - _goal[7]) * (_goal[1] - _goal[7]));
        }
    }
    return l3;
}


/**
 * 计算无人机探查目标需要往复几次实现全覆盖   目标长度
 * @param F         无人机侦察角
 * @param h         无人机飞行高度
 * @param _goal     目标数据：{西南、东南、东北、西北}
 * @return          {往复次数，路径长度}
 */
tuple<int, double> Example::calcu_cycle_count(double F, int h, vector<int> _goal) {
    // 侦察视角宽度
    double uav_d = 2 * h * tan(F / 2);

    int A1 = _goal[4] - _goal[1];
    int B1 = _goal[0] - _goal[3];
    double C1 = _goal[3] * (_goal[1] - _goal[4]) - _goal[4] * (_goal[0] - _goal[3]);

    int A2 = _goal[10] - _goal[7];
    int B2 = _goal[6] - _goal[9];
    double C2 = _goal[9] * (_goal[7] - _goal[10]) - _goal[10] * (_goal[6] - _goal[9]);

    if (A1 == 0 && B1 != 0) {
        C2 = (double) B2 / B1 * C2;
    } else if (B1 == 0 && A1 != 0) {
        C2 = (double) A1 / A2 * C2;
    }
    double goal_d_1 = fabs(C2 - C1) / sqrt(A1 * A1 + B1 * B1);


    A1 = _goal[10] - _goal[1];
    B1 = _goal[0] - _goal[9];
    C1 = _goal[9] * (_goal[1] - _goal[10]) - _goal[10] * (_goal[0] - _goal[9]);
    A2 = _goal[7] - _goal[4];
    B2 = _goal[3] - _goal[6];
    C2 = _goal[6] * (_goal[4] - _goal[7]) - _goal[7] * (_goal[3] - _goal[6]);

    if (A1 == 0 && B1 != 0) {
        C2 = (double) B2 / B1 * C2;
    } else if (B1 == 0 && A1 != 0) {
        C2 = (double) A1 / A2 * C2;
    }
    double goal_d_2 = abs(C2 - C1) / sqrt(A1 * A1 + B1 * B1);

    double goal_d = goal_d_1 < goal_d_2 ? goal_d_1 : goal_d_2;
    double goal_length = goal_d_1 < goal_d_2 ? goal_d_2 : goal_d_1;

    // 判断double类型的数据，使用一个极小值 eps = 1e-6 辅助
    int cycle_count = 0;
    if (uav_d > goal_d) {
        cycle_count = 1;
    } else {
        cycle_count = (int) ceil(goal_d / uav_d);
    }
    return {cycle_count, goal_length};
}
// ********************************* private methods end *****************************



// ******************************* getter and setter *******************************
int Example::getBreakpointNum() const {
    return breakpointNum;
}

void Example::setBreakpointNum(int breakpointNum) {
    Example::breakpointNum = breakpointNum;
}

int Example::getPointNum() const {
    return pointNum;
}

void Example::setPointNum(int pointNum) {
    Example::pointNum = pointNum;
}

int Example::getParticleNum() const {
    return particleNum;
}

void Example::setParticleNum(int particleNum) {
    Example::particleNum = particleNum;
}

int Example::getEpochs() const {
    return epochs;
}

void Example::setEpochs(int epochs) {
    Example::epochs = epochs;
}

const string &Example::getInterpType() const {
    return interpType;
}

void Example::setInterpType(const string &interpType) {
    Example::interpType = interpType;
}

int Example::getSeed() const {
    return seed;
}

void Example::setSeed(int seed) {
    Example::seed = seed;
}

int Example::getKv() const {
    return kv;
}

void Example::setKv(int kv) {
    Example::kv = kv;
}

int Example::getVisual() const {
    return visual;
}

void Example::setVisual(int visual) {
    Example::visual = visual;
}

const vector<Obstacle> &Example::getObstacles() const {
    return obstacles;
}

void Example::setObstacles(const vector<Obstacle> &obstacles) {
    Example::obstacles = obstacles;
}