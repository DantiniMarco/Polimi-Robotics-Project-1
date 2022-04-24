import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# robot standard values
l = 0.2
w = 0.169
r = 0.07
gear_ratio = 5.0

# read bag file (relative directory)
b = bagreader('../../../bags/dataset_bag3.bag');

# read topic
odo = b.message_by_topic('recorder')
# memorize data read from bag in a pandas Dataframe
odo_csv = pd.read_csv(odo)

pose_x = np.array(odo_csv["pose_x"])
pose_y = np.array(odo_csv["pose_y"])
pose_theta = np.array(odo_csv["pose_theta"])

"""orient_x = np.array(odo_csv["pose.orientation.x"])
orient_y = np.array(odo_csv["pose.orientation.y"])
orient_z = np.array(odo_csv["pose.orientation.z"])
orient_w = np.array(odo_csv["pose.orientation.w"])

siny_cosp = 2 * np.add(np.multiply(orient_w, orient_z), np.multiply(orient_x, orient_y))
cosy_cosp = 1 - 2 * np.add(np.multiply(orient_y, orient_y), np.multiply(orient_z, orient_z))
angles_yaw = np.arctan2(siny_cosp, cosy_cosp) # yaw (z-axis rotation)
"""

# read wheel velocity from bag file
"""wheel_states = b.message_by_topic("/wheel_states")
# memorize data in a pandas Dataframe
wheel_csv = pd.read_csv(wheel_states)
"""

w1 = np.array(odo_csv["w1"])
w2 = np.array(odo_csv["w2"])
w3 = np.array(odo_csv["w3"])
w4 = np.array(odo_csv["w4"])
print(odo_csv)
t_s = np.add(1000000000 * np.array(odo_csv["header.stamp.secs"]), np.array(odo_csv["header.stamp.nsecs"])) / 1000000000

# compute robot velocities
vx = np.add(np.add(np.add(w1, w2), w3), w4) * r / 4.0 / 60.0 / gear_ratio
vy = np.add(np.add(np.add(-1 * w1, w2), w3), -1 * w4) * r / 4.0 / 60.0 / gear_ratio
omega = np.add(np.add(np.add(-1 * w1, w2), -1 * w3), w4) * (r / 4.0 / (l + w)) / 60.0 / gear_ratio
theta_hat = np.zeros(len(t_s))
current_x = np.zeros(len(t_s))
current_y = np.zeros(len(t_s))
current_theta = np.zeros(len(t_s))

#odometry computations

delta_t = np.zeros(len(t_s))
delta_t[0] = 0
for t in range(1, len(t_s)):
    delta_t[t] = t_s[t] - t_s[t-1]
    theta_hat = current_theta[t-1] + omega[t] * delta_t[t] / 2.0
    current_x[t] = current_x[t-1] + (vx[t] * np.cos(theta_hat) - vy[t] * np.sin(theta_hat)) * delta_t[t]
    current_y[t] = current_y[t-1] + (vx[t] * np.sin(theta_hat) + vy[t] * np.cos(theta_hat)) * delta_t[t]
    current_theta[t] = current_theta[t-1] + omega[t] * delta_t[t] 


t_s = t_s - t_s[0]; # time scaling factor

plt.title("robot odometry")
plt.xlabel("time [s]")
plt.ylabel("robot pose x")
plt.show()
