import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import Circle, Ellipse, Rectangle, Polygon

# from etc import *
from data_one import *
# 设置字体
myfont = matplotlib.font_manager.FontProperties(fname=r'D:/Program Files (x86)/GB2312/┐м╠х_GB2312.ttf')

def plot_goal(ax, targets):
    '''
    绘制目标
    :param ax:
    :return:
    '''

    for target in targets:
        if len(target) > 3:
            element = Rectangle((target[0], target[1]), target[3] - target[0], target[10] - target[1], fc='wheat',
                                ec=None)

            ax.add_patch(element)

    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_aspect('equal', adjustable='box')
    # ax.set_xticks([])
    # ax.set_yticks([])
    ax.set_xlim(0, 200)
    ax.set_ylim(0, 200)


def plot_obs(ax, obs):
    '''
    绘制障碍
    '''
    for i in range(len(obs)):
        data = obs[i]
        type_name, x, y = data[:3]

        if (type_name == 'Circle'):
            r = data[3]
            element = Circle((x, y), r, fc='blue', ec=None)

        elif (type_name == 'Ellipse'):
            theta, b, e = data[3:6]
            theta = theta * 180.0 / np.pi
            b = 2 * b
            a = b / np.sqrt(1.0 - e ** 2)
            element = Ellipse((x, y), a, b, theta, fc='blue', ec=None)
        elif (type_name == 'Convex'):
            V = data[3]
            element = Polygon(V, closed=True, fc='blue', ec=None)

        ax.add_patch(element)
        ax.plot(x, y, 'x', ms=8, c='red')

    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_aspect('equal', adjustable='box')
    # ax.set_xticks([])
    # ax.set_yticks([])
    ax.set_xlim(0, 200)
    ax.set_ylim(0, 200)


def plot_path(ax, res, starts, goals):
    '''
    绘制路径
    '''
    for i in range(len(res)):
        Px = res[i][3]
        Py = res[i][4]
        X = res[i][0]
        nPts = len(X) // 2
        if i is 0:
            ax.plot(Px[0, :], Py[0, :], lw=0.50, c='#20C5DE')  # 路径
            ax.plot(X[:nPts], X[nPts:], 'o', ms=6, c='#20C5DE')  # 断点
        if i is 1:
            ax.plot(Px[0, :], Py[0, :], lw=0.50, c='#0B2DDE')  # 路径
            ax.plot(X[:nPts], X[nPts:], 'D', ms=6, c='#0B2DDE')  # 断点
        if i is 2:
            ax.plot(Px[0, :], Py[0, :], lw=0.50, c='#DE1711')  # 路径
            ax.plot(X[:nPts], X[nPts:], '<', ms=6, c='#DE1711')  # 断点
        if i is 3:
            ax.plot(Px[0, :], Py[0, :], lw=0.50, c='#C8B40E')  # 路径
            ax.plot(X[:nPts], X[nPts:], 'h', ms=6, c='#C8B40E')  # 断点
        ax.plot(starts[i][0], starts[i][1], 'o', ms=8, c='r')  # 起始点
        goal_x = (goals[i][0] + goals[i][3]) / 2
        goal_y = (goals[i][1] + goals[i][7]) / 2
        ax.plot(goal_x, goal_y, '*', ms=8, c='r')  # 目标点


def plot_anno(flag, axs, plt):
    '''
    绘制说明
    :param ax:
    :param plt:
    :return:
    '''
    plt.xlabel("x/km")  # x轴上的名字
    plt.ylabel("y/km")  # y轴上的名字
    axs.plot(5, 190, 'o', ms=8, c='r')  # 基站
    axs.text(10, 188, "----基站", fontsize=20, fontproperties=myfont)
    axs.plot(5, 180, '*', ms=8, c='r')  # 目标点
    axs.text(10, 178, "----目标", fontsize=20, fontproperties=myfont)
    axs.plot(5, 170, 'x', ms=8, c='r')  # 目标点
    axs.text(10, 168, "----障碍", fontsize=20, fontproperties=myfont)
    if flag:
        axs.plot(160, 40, 'o', ms=6, c='#20C5DE')  # UAV1
        axs.text(165, 37, "----UAV1", fontsize=20, fontproperties=myfont)
        axs.plot(160, 30, 'D', ms=6, c='#0B2DDE')  # UAV2
        axs.text(165, 27, "----UAV2", fontsize=20, fontproperties=myfont)
        axs.plot(160, 20, '<', ms=6, c='#DE1711')  # UAV3
        axs.text(165, 17, "----UAV3", fontsize=20, fontproperties=myfont)
        axs.plot(160, 10, 'h', ms=6, c='#C8B40E')  # UAV4
        axs.text(165, 7, "----UAV4", fontsize=20, fontproperties=myfont)


def get_circle(x, y, z, r, h):
    '''
    球   获得切面半径
    :param x: x
    :param y: y
    :param z: z
    :param r: 半径
    :param h: 高度
    :return:
    '''
    real_r = np.sqrt(r ** 2 - (h - z) ** 2)  # 切面半径
    return x, y, real_r  # 返回一个危险区域，主要是 real_r


def get_mountain_circle(x, y, z, r, h):
    '''
    圆锥   获得切面半径
    :param x: x
    :param y: y
    :param z: z
    :param r: 半径
    :param h: 高度
    :return:
    '''
    real_r = r * ((z - (h - z)) / z)
    return x, y, real_r


def get_weather_circle(x, y, z, r, h):
    '''
    圆柱   获得切面半径
    :param x: x
    :param y: y
    :param z: z
    :param r: 半径
    :param h: 高度
    :return:
    '''
    return x, y, r


def centroid(V):
    '''
    获取多边形的中心点
    :param V:
    :return:
    '''
    V = np.asarray(V)
    nPts = len(V)

    xc = 0.0  # Centroid x-coordinate
    yc = 0.0  # Centroid y-coordinate
    A = 0.0  # Polygon area

    for i in range(nPts):
        d = V[i - 1][0] * V[i][1] - V[i][0] * V[i - 1][1]
        xc += (V[i - 1][0] + V[i][0]) * d
        yc += (V[i - 1][1] + V[i][1]) * d
        A += d

    A = A / 2.0
    xc = xc / (6.0 * A)
    yc = yc / (6.0 * A)

    return xc, yc


def get_all_obs():
    # 返回数据是一个列表
    obs = []
    # 得到每一项
    for type_name, datas in obstacle.items():
        # 判断得到的障碍是什么，不同类型的障碍，计算切面圆的方式不同
        if type_name in ["radar", "gun", "missile"]:
            for data in range(len(datas)):
                x, y, real_r = get_circle(datas[data][0], datas[data][1], datas[data][2], datas[data][3], start[2])
                # 将得到的数据追加到列表中
                obs.append(["cir", x, y, real_r])
        elif type_name is "mountain":
            for data in range(len(datas)):
                x, y, real_r = get_mountain_circle(datas[data][0], datas[data][1], datas[data][2], datas[data][3],
                                                   start[2])
                obs.append(["cir", x, y, real_r])
        elif type_name is "weather":
            for data in range(len(datas)):
                x, y, real_r = get_weather_circle(datas[data][0], datas[data][1], datas[data][2], datas[data][3],
                                                  start[2])
                obs.append(["cir", x, y, real_r])
        elif type_name is "no_fly":
            # 这个数据类型需要额外处理
            for data in range(len(datas)):
                # 再申请一个列表
                V = []
                # 步长为2
                for d in range(0, len(datas[data]), 2):
                    V.append((datas[data][d], datas[data][d + 1]))
                V = np.asarray(V)
                x, y = centroid(V)
                obs.append(["con", x, y, V])
    return obs


def add_circle_obs(x, y, r, Kv):
    '''
    增加圆形障碍
    :param x: x
    :param y: y
    :param r: 半径
    :param Kv: 比例尺
    :return:
    '''
    data = ("Circle", x, y, r, Kv)
    return data


def add_convex(x, y, V, Kv):
    '''
    添加禁飞区
    :param x: 中心点x
    :param y: 中心点y
    :param V: 点坐标
    :param Kv: 比例尺
    :return:
    '''
    data = ("Convex", x, y, V, Kv)
    return data


def plot_all(end_res):
    '''
    :param end_res:
    :return:
    '''
    obs_ = []
    targets = []
    obs = get_all_obs()
    starts = []
    goals_ = []
    for i in range(len(obs)):
        if obs[i][0] is "cir":
            obs_.append(add_circle_obs(x=obs[i][1], y=obs[i][2], r=obs[i][3], Kv=100))
        elif obs[i][0] is "con":
            obs_.append(add_convex(x=obs[i][1], y=obs[i][2], V=obs[i][3], Kv=100))

    for name, goal in goals.items():
        for i in range(len(goal)):
            targets.append(goal[i])

    for start in uavs_start:
        starts.append(start[:2])

    for name, goal in goals.items():
        for i in range(len(goal)):
            goals_.append(goal[i][:12])

    # 绘图（分配后的）
    fig, axs = plt.subplots()
    plot_goal(axs, targets)
    plot_obs(axs, obs_)
    plot_path(axs, end_res, starts, goals_)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    plot_anno(True, axs, plt)
    plt.show()
