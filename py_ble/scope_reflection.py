import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.ticker import ScalarFormatter

# 设置字体为 Times New Roman
plt.rc('font', family='Times New Roman')

# Use 'TkAgg' backend to open the plot in a new window
matplotlib.use('TkAgg')

# Load the data
file_path = 'scope_data.csv'
data = pd.read_csv(file_path)

# Filter data starting from wavelength 450
data = data[data['wavelength'] >= 450]

# Normalize by integration time (10.14)
data['li_scope'] = data['li_scope'] / 10.14
data['white_scope'] = data['white_scope'] / 10.14

# Create the figure and subplots with IEEE-compliant size (8.5 cm x 13 cm)
fig, (ax2, ax1) = plt.subplots(2, 1, figsize=(9, 9), sharex=True)

# Plot Spectral Magnitude vs Wavelength (Top subplot)
ax2.plot(data['wavelength'], data['li_scope'], label='Ankle', linestyle='-')
ax2.plot(data['wavelength'], data['white_scope'], label='PTFE', linestyle='-')
ax2.set_ylabel('Relative Spectral Response', fontsize=24)
ax2.legend(fontsize=16)
# ax2.set_title('Spectral Magnitude of Reference Material and Ankle Ligament vs Wavelength', fontsize=10)
ax2.grid(True, linestyle='--', linewidth=1)

# Plot Reflectance vs Wavelength (Bottom subplot)
ax1.plot(data['wavelength'], data['li_ref'], label='Ankle', linestyle='-')
ax1.plot(data['wavelength'], data['white_ref'], label='PTFE', linestyle='-')
ax1.set_xlabel('Wavelength (nm)', fontsize=24)
ax1.set_ylabel('Reflectance (%)', fontsize=24)
ax1.legend(fontsize=16)
# ax1.set_title('Reflectance of Reference Material and Ankle Ligament vs Wavelength', fontsize=10)
ax1.grid(True, linestyle='--', linewidth=1)

# Set x-axis ticks to 450, 500, 550, 600, ...
xticks = list(range(450, int(data['wavelength'].max()) + 1, 50))
ax1.set_xticks(xticks)
ax1.set_xticklabels([str(x) for x in xticks], fontsize=16)
ax2.set_xticks(xticks)
ax2.set_xticklabels([str(x) for x in xticks], fontsize=16)
# 设置 y 轴刻度字体大小
ax1.tick_params(axis='y', labelsize=16)
ax2.tick_params(axis='y', labelsize=16)


# Remove scientific notation on x-axis
ax1.xaxis.set_major_formatter(ScalarFormatter())
ax2.xaxis.set_major_formatter(ScalarFormatter())

# Adjust layout for better spacing
plt.tight_layout()

# Save as high-resolution PNG (300 DPI) for IEEE paper submission
plt.savefig('spectral_analysis.png', dpi=600, bbox_inches='tight')

# Show the plot in a new window
plt.show()