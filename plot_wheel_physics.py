import matplotlib.pyplot as plt
import csv
import numpy as np

time = []
w = []
alpha = []
a_point = []

try:
    with open('wheel_physics_results.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            time.append(float(row['t']))
            w.append([float(row['w_x']), float(row['w_y']), float(row['w_z'])])
            alpha.append([float(row['alpha_x']), float(row['alpha_y']), float(row['alpha_z'])])
            a_point.append([float(row['a_point_x']), float(row['a_point_y']), float(row['a_point_z'])])
except FileNotFoundError:
    print("Error: wheel_physics_results.csv not found.")
    exit(1)

w = np.array(w)
alpha = np.array(alpha)
a_point = np.array(a_point)

fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# 1. Angular Velocity
axs[0].plot(time, w[:, 0], label='w_x')
axs[0].plot(time, w[:, 1], label='w_y')
axs[0].plot(time, w[:, 2], label='w_z', linewidth=2)
axs[0].set_ylabel('Angular Vel (rad/s)')
axs[0].set_title('Angular Velocity (Body)')
axs[0].legend()
axs[0].grid(True)
axs[0].text(time[len(time)//2], 10.5, 'Target w_z = 10 rad/s', horizontalalignment='center', color='green')

# 2. Point Acceleration (Centrifugal)
axs[1].plot(time, a_point[:, 0], label='a_point_x', linewidth=2)
axs[1].plot(time, a_point[:, 1], label='a_point_y')
axs[1].plot(time, a_point[:, 2], label='a_point_z')
axs[1].set_ylabel('Acceleration (m/s^2)')
axs[1].set_title('Point Acceleration (Lever Arm r=0.3m)')
axs[1].legend()
axs[1].grid(True)
axs[1].text(time[len(time)//2], -28, 'Expected a_x = -30 m/s^2', horizontalalignment='center', color='green')

# 3. Angular Acceleration
axs[2].plot(time, alpha[:, 0], label='alpha_x')
axs[2].plot(time, alpha[:, 1], label='alpha_y')
axs[2].plot(time, alpha[:, 2], label='alpha_z')
axs[2].set_ylabel('Angular Accel (rad/s^2)')
axs[2].set_title('Angular Acceleration (Body)')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig('wheel_physics_plot.png')
print("Plot saved to wheel_physics_plot.png")
