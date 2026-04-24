import pandas as pd
import matplotlib.pyplot as plt

file_path = "/home/oliver/Rob4-Gr444-Projekt/ros2_ws/logs/temp.csv"

# TurtleBot3 Burger wheel separation
wheel_base_m = 0.160

# Load CSV
df = pd.read_csv(file_path)

# Convert relevant columns to numeric
for col in [
    "elapsed_sec",
    "cmd_angular_speed",
    "imu_ang_vel_z",
    "joint_state_time",
    "left_wheel_velocity",
    "right_wheel_velocity",
]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Keep rows with valid elapsed time
df = df.dropna(subset=["elapsed_sec"]).copy()
df = df.sort_values("elapsed_sec").reset_index(drop=True)

# -----------------------------
# IMU data
# -----------------------------
df_imu = df.dropna(subset=["imu_ang_vel_z"]).copy()

# -----------------------------
# Wheel-velocity-based yaw rate
# -----------------------------
df_wheels = df.dropna(
    subset=["joint_state_time", "left_wheel_velocity", "right_wheel_velocity"]
).copy()

df_wheels = df_wheels.sort_values("joint_state_time").reset_index(drop=True)

# Remove duplicate joint_state_time rows if they exist
df_wheels = df_wheels.drop_duplicates(subset=["joint_state_time"], keep="last").copy()

# Time axis relative to first valid joint-state timestamp
df_wheels["wheel_elapsed_sec"] = (
    df_wheels["joint_state_time"] - df_wheels["joint_state_time"].iloc[0]
)

# Yaw rate from wheel velocities
df_wheels["yaw_rate_wheels"] = (
    df_wheels["right_wheel_velocity"] - df_wheels["left_wheel_velocity"]
) / wheel_base_m

# -----------------------------
# Plot 1: IMU yaw rate only
# -----------------------------
plt.figure()
plt.plot(df_imu["elapsed_sec"], df_imu["imu_ang_vel_z"])
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")
plt.title("IMU Yaw Rate over Time")
plt.grid()

# -----------------------------
# Plot 2: Commanded vs IMU yaw rate
# -----------------------------
plt.figure()
plt.plot(df["elapsed_sec"], df["cmd_angular_speed"], label="Commanded yaw rate")
plt.plot(df_imu["elapsed_sec"], df_imu["imu_ang_vel_z"], label="Measured IMU yaw rate")
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")
plt.title("Commanded vs Measured IMU Yaw Rate")
plt.grid()
plt.legend()

# -----------------------------
# Plot 3: Wheel-velocity-based yaw rate
# -----------------------------
plt.figure()
plt.plot(df_wheels["wheel_elapsed_sec"], df_wheels["yaw_rate_wheels"])
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")
plt.title("Yaw Rate from Left/Right Wheel Velocities")
plt.grid()

# -----------------------------
# Plot 4: Comparison
# -----------------------------
plt.figure()
plt.plot(df["elapsed_sec"], df["cmd_angular_speed"], label="Commanded yaw rate")
plt.plot(df_imu["elapsed_sec"], df_imu["imu_ang_vel_z"], label="IMU yaw rate")
plt.plot(df_wheels["wheel_elapsed_sec"], df_wheels["yaw_rate_wheels"], label="Wheel-velocity yaw rate")
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")
plt.title("Yaw Rate Comparison")
plt.grid()
plt.legend()

plt.show()