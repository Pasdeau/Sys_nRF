import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import matplotlib

matplotlib.use('TkAgg')

# 设置动态窗口
ENABLE_DYNAMIC_WINDOW = True  # 设为 False 则使用静态窗口
DYNAMIC_WINDOW_SIZE = 1       # 动态窗口大小（秒）
STATIC_WINDOW_START = 90      # 静态窗口起始时间（秒）
STATIC_WINDOW_END = 92        # 静态窗口结束时间（秒）

# CSV 文件路径
csv_path = 'test_data.csv'

# 创建图像和子图
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 15), sharex=True)

def dynamic_plot(i):
    try:
        data = pd.read_csv(csv_path)

        if 'Time' not in data.columns:
            print("Error: Missing 'Time' column in the CSV file.")
            return

        required_columns = ['LED1', 'LED2', 'LED3', 'LED4']
        for col in required_columns:
            if col not in data.columns:
                print(f"Error: Missing column '{col}' in the CSV file.")
                return

        # 调整时间列，使其从 0 开始
        data['Time'] = data['Time'] - data['Time'].iloc[0]

        # 选择绘图数据窗口
        if ENABLE_DYNAMIC_WINDOW:
            max_time = data['Time'].iloc[-1]
            data = data[data['Time'] >= max_time - DYNAMIC_WINDOW_SIZE]
        else:
            data = data[(data['Time'] >= STATIC_WINDOW_START) & (data['Time'] <= STATIC_WINDOW_END)]

        # 提取时间和 LED 信号数据
        x = data['Time']
        signal_led1 = data['LED1']
        signal_led2 = data['LED2']
        signal_led3 = data['LED3']
        signal_led4 = data['LED4']

        # 清除当前图像
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

        # 绘制信号曲线并添加点标记
        ax1.plot(x, signal_led1, label='LED1 (670nm)', color='blue', linewidth=1, marker='o', markersize=3)
        ax1.legend(loc='upper left')
        ax1.set_ylabel('Signal')
        ax1.set_title('LED1 (670nm)')

        ax2.plot(x, signal_led2, label='LED2 (850nm)', color='red', linewidth=1, marker='o', markersize=3)
        ax2.legend(loc='upper left')
        ax2.set_ylabel('Signal')
        ax2.set_title('LED2 (850nm)')

        ax3.plot(x, signal_led3, label='LED3 (950nm)', color='green', linewidth=1, marker='o', markersize=3)
        ax3.legend(loc='upper left')
        ax3.set_ylabel('Signal')
        ax3.set_title('LED3 (950nm)')

        ax4.plot(x, signal_led4, label='LED4 (1300nm)', color='purple', linewidth=1, marker='o', markersize=3)
        ax4.legend(loc='upper left')
        ax4.set_ylabel('Signal')
        ax4.set_xlabel('Time (s)')
        ax4.set_title('LED4 (1300nm)')

        # 设置 x 轴范围
        if ENABLE_DYNAMIC_WINDOW:
            ax1.set_xlim(max(0, max_time - DYNAMIC_WINDOW_SIZE), max_time)
            ax2.set_xlim(max(0, max_time - DYNAMIC_WINDOW_SIZE), max_time)
            ax3.set_xlim(max(0, max_time - DYNAMIC_WINDOW_SIZE), max_time)
            ax4.set_xlim(max(0, max_time - DYNAMIC_WINDOW_SIZE), max_time)
        else:
            ax1.set_xlim(STATIC_WINDOW_START, STATIC_WINDOW_END)
            ax2.set_xlim(STATIC_WINDOW_START, STATIC_WINDOW_END)
            ax3.set_xlim(STATIC_WINDOW_START, STATIC_WINDOW_END)
            ax4.set_xlim(STATIC_WINDOW_START, STATIC_WINDOW_END)

        plt.tight_layout()

    except pd.errors.EmptyDataError:
        print("Error: The CSV file is empty. Waiting for data...")
    except FileNotFoundError:
        print(f"Error: File '{csv_path}' not found. Please check the file path.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

# 设置动画更新数据
ani = FuncAnimation(fig, dynamic_plot, interval=100, cache_frame_data=False)

# 显示图像
plt.show()
