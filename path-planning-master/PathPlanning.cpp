#include "PathPlanning.h"

/**
 * 无参构造器
 */
PathPlanning::PathPlanning() {};

/**
 * 有参构造
 * @param limits 空间限制
 */
PathPlanning::PathPlanning(const vector<int>& limits) {
    this->limits = limits;
}

/**
 * 有参构造器
 * @param start
 * @param goal
 * @param limits
 */
PathPlanning::PathPlanning(vector<int> start, vector<int> goal, vector<int> limits) {
    // 提取(x, y)
    this->start.assign(start.begin(), start.begin() + 2);
    // 所有数据都提取
    this->goal.assign(goal.begin(), goal.begin() + 12);
    // 和goal数据一致
    this->target = goal;
    // 空间限制全部都要
    this->limits.assign(limits.begin(), limits.begin() + 5);
}

/**
 * 重载运算符，输出类本身信息
 * @param os
 * @param pp
 * @return
 */
ostream &operator<<(ostream &os, const PathPlanning &pp) {
    os << "\nPathPlanning object"
       << "\n- start = [" << pp.start[0] << ", " << pp.start[1] << "]"
       << "\n- goal = [" << pp.goal[0] << ", " << pp.goal[1]
       << "\n- limits = " << pp.limits[0] << ", " << pp.limits[1] << ", " << pp.limits[2] << ", " << pp.limits[3]
       << "\n- number of obstacles = " << pp.obs.size();
    return os;
}

/**
 * 障碍信息
 */
void PathPlanning::obs_info() {

    size_t obs_count = obs.size();

    if (obs_count > 0) {
        cout << "<------------------------Obstacles-------------------------->\n";
    } else {
        cout << "<-----------------------No Obstacles------------------------>\n";
    }

    for (size_t i = 0; i < obs_count; ++i) {
        auto &data = obs[i];
        /*
         * type_name：障碍类型
         * 中心点：(x, y)
         * */
        string type_name;
        double x, y, r, Kv;

        /*
         * 首先，赋值无区别参数：类型、坐标(x, y)
         * */
        type_name = data.type_name;
        x = data.x;
        y = data.y;
        r = data.real_r;
        Kv = data.kv;
        // 圆类型
        if (type_name == "Circle") {
            cout << "\n" << type_name
                 << "\n- centroid = (" << x << ", " << y << ")"
                 << "\n- radius = " << r
                 << "\n- scaling factor = " << Kv << "\n";
        } else if (type_name == "Ellipse") { // 椭圆类型
            double theta, b, e;
            // get函数，获取元组的数据引用，get<3>(data): 获取data的第3个元素
//            theta = get<3>(data) * 180.0 / M_PI;
//            b = get<4>(data);
//            e = get<5>(data);
//            Kv = get<6>(data);

            double a = b / sqrt(1.0 - e * e);
            std::cout << "\n" << type_name
                      << "\n- centroid = (" << x << ", " << y << ")"
                      << "\n- rotation from x-axis= " << theta
                      << "\n- semi-major axis = " << a
                      << "\n- semi-minor axis = " << b
                      << "\n- scaling factor = " << Kv << "\n";
        }
    }

}

/**
 * 添加圆形障碍（其实这里是将type_name更改，或许有利于绘图）
 * @param x
 * @param y
 * @param r     切面圆半径
 * @param Kv    比例尺
 */
void PathPlanning::add_circle_obs(double x, double y, double r, double Kv) {
    this->obs.push_back({"Circle", x, y, r, Kv});
}

/**
 * 添加椭圆切面
 * @param x
 * @param y
 * @param theta
 * @param a
 * @param b
 * @param Kv
 */
void PathPlanning::add_ellipse_obs(double x, double y, double theta, double a, double b, double Kv) {
    double e = sqrt(1.0 - (b * b) / (a * a));
    // obs.emplace_back("Ellipse", x, y, theta, b, e, Kv);
}

/**
 * 添加不规则图形障碍
 * @param x
 * @param y
 * @param V     各个端点的坐标
 * @param Kv
 */
void PathPlanning::add_convex(double x, double y, vector<vector<int>> V, double Kv) {
    obs.push_back({"Convex", x, y, 0.0, Kv, V});
}

/**
 * 根据 id 删除障碍
 * @param idx
 */
void PathPlanning::remove_obs(size_t idx) {
    if (idx < obs.size()) {
        obs.erase(obs.begin() + idx);
        cout << "删除成功\n";
    } else {
        cout << "没有该障碍\n";
    }
}

/**
 * 优化路径
 * @param n_pts     粒子包含的点个数，也就是维度/2
 * @param _ns       计算插值点个数
 * @param n_pop     粒子数量
 * @param epochs    迭代次数
 * @param k         全局
 * @param phi       计算权重
 * @param vel_fact  计算最大最小速度
 * @param conf_type 约束类型
 * @param int_var   指定将哪个变量视为整数的索引列表
 * @param normalize 指定是否应该对搜索空间进行规范化（以提高收敛性）
 * @param rad       判断粒子是否符合条件
 * @param finterp   插值方法
 * @param xinit     粒子的初始值
 * @return 断点坐标  总路径长度  路径中有几个点在障碍物内  插值的x坐标  插值的y坐标
 */
tuple<vector<double>, double, int, vector<double>, vector<double>>
PathPlanning::optimize(int n_pts, int _ns, int n_pop, int epochs, int k, double phi, double vel_fact, string conf_type,
                       const string& int_var, bool normalize, double rad, const string& finterp, vector<vector<int>> xinit) {
    // 指定空间边界
    int nVar = 2 * n_pts;
    vector<int> LB(nVar, 0);
    vector<int> UB(nVar, 0);
    for (int i = 0; i < nVar; i++) {
        if (i < n_pts) {
            LB[i] = this->limits[0];
            UB[i] = this->limits[1];
        } else {
            LB[i] = this->limits[2];
            UB[i] = this->limits[3];
        }
    }

    // 构造参数
    // 其实位置
    vector<int> start_vec;
    start_vec.push_back(start[0]);
    start_vec.push_back(start[1]);
    vector<vector<int>> ss(n_pop, start_vec);
    // 目标位置
    vector<int> goal_vec;
    goal_vec.push_back(goal[0]);
    goal_vec.push_back(goal[1]);
    vector<vector<int>> gg(n_pop, goal_vec);

    Args args(ss, gg, this->obs, _ns, finterp);

    /*
     * 参数声明：
     * best_pos：种群最优值
     * info：{最优cost，最优粒子索引，小于rad的粒子数}
     * */
    vector<double> best_pos;
    tuple<double, int, int> info;

    // 使用PSO算法优化 得到最优路径{内部断点，info}
    tie(best_pos, info) = PSO(LB, UB, n_pop, epochs, k, phi, vel_fact, conf_type, int_var, normalize, rad, args,
                              xinit);

    /*
     * 参数声明：
     * L: 总路径长度
     * count: 路径中有几个点在障碍物内
     * Px: 插值的x坐标
     * Py: 插值的y坐标
     * */
    double L;
    int count;
    vector<double> Px, Py;
    Args args2(this->start, this->goal, this->obs, _ns, finterp);
    tie(L, count, Px, Py) = calc_path_length(best_pos, args2);

    // 更新结果
    this->res = {best_pos, L, count, Px, Py};

    return {best_pos, L, count, Px, Py};
}

/*
 ********************************* getter and setter ***********************************
 * */
const vector<int> &PathPlanning::getStart() const {
    return start;
}

void PathPlanning::setStart(const vector<int> &start) {
    PathPlanning::start = start;
}

const vector<int> &PathPlanning::getGoal() const {
    return goal;
}

void PathPlanning::setGoal(const vector<int> &goal) {
    PathPlanning::goal = goal;
}

const vector<int> &PathPlanning::getTarget() const {
    return target;
}

void PathPlanning::setTarget(const vector<int> &target) {
    PathPlanning::target = target;
}

const vector<int> &PathPlanning::getLimits() const {
    return limits;
}

void PathPlanning::setLimits(const vector<int> &limits) {
    PathPlanning::limits = limits;
}

const vector<Obstacle> &PathPlanning::getObs() const {
    return obs;
}

void PathPlanning::setObs(const vector<Obstacle> &obs) {
    PathPlanning::obs = obs;
}

const Result &PathPlanning::getRes() const {
    return res;
}

void PathPlanning::setRes(const Result &res) {
    PathPlanning::res = res;
}








