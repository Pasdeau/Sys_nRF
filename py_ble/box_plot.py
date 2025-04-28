import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import FuncFormatter
from scipy.stats.mstats import winsorize

file_path = 'ref_data.csv'
sorted_data = pd.read_csv(file_path)
plt.rc('font', family='Times New Roman')

# sorted_data = sorted_data[sorted_data['wavelength'] >= 450]  # Filter out wavelength < 450

# Compute absolute left vs right percentage differences
sorted_data['Percentage_Difference_1'] = abs((sorted_data['1L'] - (sorted_data['1R'] + 1.4)) / sorted_data['1R'] * 100)
sorted_data['Percentage_Difference_2'] = abs((sorted_data['2L'] - (sorted_data['2R'] + 0.0)) / sorted_data['2R'] * 100)
sorted_data['Percentage_Difference_3'] = abs((sorted_data['3L'] - (sorted_data['3R'] + 0.0)) / sorted_data['3R'] * 100)
sorted_data['Percentage_Difference_4'] = abs((sorted_data['4L'] - (sorted_data['4R'] - 0.2)) / sorted_data['4R'] * 100)
sorted_data['Percentage_Difference_5'] = abs((sorted_data['5L'] - (sorted_data['5R'] - 0.4)) / sorted_data['5R'] * 100)

# Define data columns to plot
columns_to_plot = ['Percentage_Difference_1', 'Percentage_Difference_2', 'Percentage_Difference_3', 'Percentage_Difference_4', 'Percentage_Difference_5']

# Winsorization
for col in columns_to_plot:
    sorted_data[col] = winsorize(sorted_data[col], limits=[0.1, 0.1])  # Limit top and bottom 1%

# Create figure without specifying size (use default size)
plt.figure(figsize=(7, 4))

plt.rcParams.update({
    'font.size': 6,          # 主字体大小
    'axes.titlesize': 8,    # 标题大小
    'axes.labelsize': 6,     # 轴标签大小
    'xtick.labelsize': 7,    # X轴刻度大小
    'ytick.labelsize': 7,    # Y轴刻度大小
    'legend.fontsize': 7     # 图例字体大小
})

# Plot box plot (hide outliers)
sorted_data[columns_to_plot].boxplot(
    patch_artist=True,
    boxprops=dict(facecolor='lightblue', color='blue', linewidth=0.7),
    medianprops=dict(color='red', linewidth=0.7),
    whiskerprops=dict(linewidth=0.7),
    capprops=dict(linewidth=0.7),
    showfliers=False
)

# plt.title('Differential Analysis of Bilateral Ankle Ligaments', fontsize=20)
plt.ylabel('Différence de réflectance bilatérale', fontsize=19)
plt.xticks([1, 2, 3, 4, 5], ['Sujet 1', 'Sujet 2', 'Sujet 3', 'Sujet 4', 'Sujet 5'], fontsize=20)

# Remove percentage formatting (show raw values)
plt.gca().yaxis.set_major_formatter(FuncFormatter(lambda y, _: f'{y:.0f} % '))
plt.gca().tick_params(axis='y', labelsize=18)

# Add annotations (without percentage symbol)
for i in range(1, 6):
    plt.annotate(f'{sorted_data[columns_to_plot].median().iloc[i-1]:.2f}',
                 xy=(i, sorted_data[columns_to_plot].median().iloc[i-1]),
                 xytext=(i, sorted_data[columns_to_plot].median().iloc[i-1] + 0.25),
                 textcoords='data', va='center',
                 ha='center', color='black', fontsize=16)

# Adjust layout for better appearance
plt.tight_layout(pad=0.2)
plt.savefig('box_plotting.png', dpi=1000, bbox_inches='tight')

# Display plot
plt.show()
