import matplotlib.pyplot as plt
import csv
import numpy as np

time = []
meas = []
pred = []

try:
    with open('inertial_physics_results.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            time.append(float(row['t']))
            meas.append([float(row['meas_ax']), float(row['meas_ay']), float(row['meas_az'])])
            pred.append([float(row['pred_ax']), float(row['pred_ay']), float(row['pred_az'])])
except FileNotFoundError:
    print("Error: inertial_physics_results.csv not found.")
    exit(1)

meas = np.array(meas)
pred = np.array(pred)

fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# X Axis
axs[0].plot(time, meas[:, 0], label='Measured X', linewidth=2, linestyle='--')
axs[0].plot(time, pred[:, 0], label='Predicted X', linewidth=1, alpha=0.8)
axs[0].set_ylabel('Accel X (m/s^2)')
axs[0].set_title('Inertial Physics: Lever Arm Effect (X)')
axs[0].legend()
axs[0].grid(True)
# Expected: Centrifugal (-w^2 * r). w=100t. a = -10000 t^2 * 0.1 = -1000 t^2.
# At t=0.5, a = -1000 * 0.25 = -250.

# Y Axis
axs[1].plot(time, meas[:, 1], label='Measured Y', linewidth=2, linestyle='--')
axs[1].plot(time, pred[:, 1], label='Predicted Y', linewidth=1, alpha=0.8)
axs[1].set_ylabel('Accel Y (m/s^2)')
axs[1].set_title('Inertial Physics: Lever Arm Effect (Y)')
axs[1].legend()
axs[1].grid(True)
# Expected: Tangential (alpha * r). alpha=100. a = 100 * 0.1 = 10. Constant.

# Z Axis
axs[2].plot(time, meas[:, 2], label='Measured Z', linewidth=2, linestyle='--')
axs[2].plot(time, pred[:, 2], label='Predicted Z', linewidth=1, alpha=0.8)
axs[2].set_ylabel('Accel Z (m/s^2)')
axs[2].set_title('Inertial Physics: Lever Arm Effect (Z)')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig('inertial_physics_plot.png')
print("Plot saved to inertial_physics_plot.png")
