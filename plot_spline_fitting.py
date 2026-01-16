import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import numpy as np

# Read data
gt_data = []
opt_data = []

try:
    with open('spline_fitting_results.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            gt_data.append([float(row['gt_x']), float(row['gt_y']), float(row['gt_z'])])
            opt_data.append([float(row['opt_x']), float(row['opt_y']), float(row['opt_z'])])
except FileNotFoundError:
    print("Error: spline_fitting_results.csv not found. Run the C++ example first.")
    exit(1)

gt_data = np.array(gt_data)
opt_data = np.array(opt_data)

# 1. 3D Trajectory Plot
fig = plt.figure(figsize=(12, 6))
ax = fig.add_subplot(121, projection='3d')

ax.plot(gt_data[:, 0], gt_data[:, 1], gt_data[:, 2], label='Ground Truth', color='black', linewidth=1, linestyle='--')
ax.plot(opt_data[:, 0], opt_data[:, 1], opt_data[:, 2], label='Optimized Spline', color='red', linewidth=2, alpha=0.7)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Trajectory Comparison')
ax.legend()

# 2. Error Plot (Euclidean Distance)
ax2 = fig.add_subplot(122)
errors = np.linalg.norm(gt_data - opt_data, axis=1)
time_steps = np.arange(len(errors)) * 0.01 # Assuming 100Hz from C++ code but file doesn't strictly track time step per row uniformly if gaps existed. 
# Better to read time column if available, but here assuming sequential valid data.

ax2.plot(errors, label='Position Error', color='blue')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Error (m)')
ax2.set_title('Position Error (Euclidean)')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.savefig('spline_fitting_plot.png')
print("Plot saved to spline_fitting_plot.png")
