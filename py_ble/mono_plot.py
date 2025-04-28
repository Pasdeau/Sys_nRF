import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import matplotlib

matplotlib.use('TkAgg')

ENABLE_DYNAMIC_WINDOW = False  # False to disable dynamic window
DYNAMIC_WINDOW_SIZE = 10  # Window size in seconds
STATIC_WINDOW_START = 45  # Start time for static window in seconds
STATIC_WINDOW_END = 80  # End time for static window in seconds

csv_path = 'test_mono.csv'

fig, ax = plt.subplots(figsize=(10, 6))  # 改为单图显示


def dynamic_plot(i):
    try:
        data = pd.read_csv(csv_path)

        # 验证列名
        required_columns = ['Time', 'Value']
        for col in required_columns:
            if col not in data.columns:
                print(f"Error: Missing column '{col}' in the CSV file.")
                return

        # 处理时间数据（可保持原逻辑）
        if not pd.api.types.is_numeric_dtype(data['Time']):
            data['Time'] = pd.to_datetime(data['Time']).astype('int64') / 1e9

        # 转换为相对时间（从0开始）
        data['Time'] = data['Time'] - data['Time'].iloc[0]

        # 窗口筛选逻辑
        if ENABLE_DYNAMIC_WINDOW:
            max_time = data['Time'].iloc[-1]
            data = data[data['Time'] >= max(0, max_time - DYNAMIC_WINDOW_SIZE)]
        else:
            data = data[(data['Time'] >= STATIC_WINDOW_START) &
                        (data['Time'] <= STATIC_WINDOW_END)]

        # 清空画布
        ax.clear()

        # 绘制单线图
        ax.plot(data['Time'], data['Value'],
                color='blue',
                linewidth=1,
                label='Sensor Value')

        # 设置图形参数
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Value')
        ax.set_title('Real-time Sensor Data')
        ax.legend(loc='upper right')
        ax.grid(True)

        # 设置坐标轴范围
        if ENABLE_DYNAMIC_WINDOW:
            ax.set_xlim(max(0, data['Time'].max() - DYNAMIC_WINDOW_SIZE),
                        data['Time'].max())
        else:
            ax.set_xlim(STATIC_WINDOW_START, STATIC_WINDOW_END)

        # 自动调整Y轴范围
        ax.set_ylim(data['Value'].min() * 0.95, data['Value'].max() * 1.05)

        plt.tight_layout()

    except pd.errors.EmptyDataError:
        print("Error: The CSV file is empty. Waiting for data...")
    except FileNotFoundError:
        print(f"Error: File '{csv_path}' not found. Please check the file path.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


ani = FuncAnimation(fig, dynamic_plot, interval=100, cache_frame_data=False)
plt.show()