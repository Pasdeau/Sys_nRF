import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import matplotlib

matplotlib.use('TkAgg')  # Changer le backend

csv_path = 'data.csv'

# 创建一个子图
fig, ax = plt.subplots(figsize=(10, 6))

def dynamic_plot(i):
    data = pd.read_csv(csv_path)

    # TEMPS --> X
    x = data['TEMPS']
    y = data['LED1']  # LED1 数据

    ax.clear()  # 清空子图
    ax.plot(x, y, label='LED1', color='blue', linewidth=1)  # 绘制曲线
    ax.legend(loc='upper left')  # 显示图例
    ax.set_xlabel('TEMPS (s)')  # X轴标签
    ax.set_ylabel('LED (mV)')  # Y轴标签
    ax.set_title('Dynamic Plot')  # 图表标题

    plt.tight_layout()  # 自动调整布局

# 动态更新图表
ani = FuncAnimation(fig, dynamic_plot, interval=1000, cache_frame_data=False)

plt.show()
