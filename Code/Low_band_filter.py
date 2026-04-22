import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- Load data ---
data = pd.read_csv("/home/jacob/Desktop/Rob4-Gr444-Projekt/ros2_ws/logs/speed_log.csv")

t = data["time"].values
yaw_raw = data["imu_yaw_rate"].values
left_wheel = data["left_wheel"].values
right_wheel = data["right_wheel"].values
t = t[:420]
yaw_raw = yaw_raw[:420]
left_wheel = left_wheel[:420]
right_wheel = right_wheel[:420]

# --- 1st order low-pass filter ---
alpha = 0.9

yaw_lpf = np.zeros_like(yaw_raw)
left_lpf = np.zeros_like(left_wheel)
right_lpf = np.zeros_like(right_wheel)

yaw_lpf[0] = yaw_raw[0]
left_lpf[0] = left_wheel[0]
right_lpf[0] = right_wheel[0]

for k in range(1, len(yaw_raw)):
    yaw_lpf[k] = alpha * yaw_lpf[k-1] + (1 - alpha) * yaw_raw[k]

for k in range(1, len(left_wheel)):
    left_lpf[k] = alpha * left_lpf[k-1] + (1 - alpha) * left_wheel[k]

for k in range(1, len(right_wheel)):
    right_lpf[k] = alpha * right_lpf[k-1] + (1 - alpha) * right_wheel[k]

# --- Moving average (yaw only) ---
window_size = 10
kernel = np.ones(window_size) / window_size
yaw_ma = np.convolve(yaw_raw, kernel, mode="same")

wheels = [(right_lpf[i]-left_lpf[i])/0.16 for i in range(right_lpf.size)]



# --- Plot ---
plt.figure(figsize=(14, 5))

# Yaw-rate
plt.subplot(1, 2, 1)
plt.plot(t, yaw_raw, label="IMU yaw rate (raw)", alpha=0.3)
plt.plot(t, yaw_lpf, label="Yaw LPF", linewidth=2)
plt.plot(t, wheels, label="wheel rad/s", linewidth=2)
plt.plot(t, yaw_ma, label="Yaw moving average", linestyle="--")
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")
plt.legend()
plt.grid()

# Wheels
plt.subplot(1, 2, 2)
plt.plot(t, left_wheel, label="Left wheel (raw)", alpha=0.3)
plt.plot(t, right_wheel, label="Right wheel (raw)", alpha=0.3)
plt.plot(t, left_lpf, label="Left wheel LPF", linewidth=2)
plt.plot(t, right_lpf, label="Right wheel LPF", linewidth=2)
plt.xlabel("Time [s]")
plt.ylabel("Wheel speed [rad/s]")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()