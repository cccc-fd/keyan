import matplotlib.pyplot as plt

# 创建数据
x = [2, 4, 5, 8]
y = [42.0121, 32.3236, 22.7491, 24.2708]

# 绘制折线图
plt.plot(x, y)

# 设置x轴和y轴标签
plt.xlabel('THREADS_NUM')
plt.ylabel('运行总时长')

# 设置图表标题
plt.title('并行线程数对运行时长的影响')

# 显示图表
plt.show()
