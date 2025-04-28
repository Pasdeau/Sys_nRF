import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pandas as pd

# 读取 CSV 文件
file_path = 'consummation_test.csv'
df = pd.read_csv(file_path)

# 时间、电流预处理
time_s = df['Timestamp(ms)'] / 1000         # 毫秒 → 秒
current_mA = df['Current(uA)'] / 1000       # 微安 → 毫安
current_mA = current_mA.clip(upper=20)      # 可自行调整裁剪上限

# 下采样
step = 100
downsampled_time = time_s[::step].reset_index(drop=True)
downsampled_current = current_mA[::step].reset_index(drop=True)

# 滑动平均
window_size = 12
smoothed_current = downsampled_current.rolling(window=window_size, center=True).mean()

# 设置字体
plt.rc('font', family='Times New Roman')
plt.rcParams.update({
    'font.size': 22,
    'axes.titlesize': 24,
    'axes.labelsize': 22,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'legend.fontsize': 18
})

# 创建图像
plt.figure(figsize=(8.5, 5.5))
plt.plot(downsampled_time, smoothed_current, linewidth=1.2, color='blue')

# ✅ 阶段分隔与标签
stage_edges = [0, 0.1, 8, 15, 31, 72, 82.5, 95.5, 96]

# 添加垂直线和文字标注
for i in range(1, len(stage_edges)-1):
    plt.axvline(x=stage_edges[i], color='red', linestyle='--', linewidth=0.8)

# 坐标轴标签等
plt.xlabel('Time (s)', fontsize=22)
plt.ylabel('Current (mA)', fontsize=22)
# plt.title('Smoothed Current Consumption Over Time', fontsize=24)
plt.grid(True)

# 保存与显示
plt.tight_layout(pad=0.3)
plt.savefig('current_plot_with_stages.png', dpi=600, bbox_inches='tight')
plt.show()