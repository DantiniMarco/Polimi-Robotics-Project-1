import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class Estimator:

    def __init__(self, bag_name):

        # robot standard values
        self.l = 0.2
        self.w = 0.169
        self.r = 0.07
        self.gear_ratio = 5.0
        # read bag file (relative directory)
        self.b = bagreader(bag_name)

        # read topic
        self.odo = self.b.message_by_topic('/recorder')
        # memorize data read from bag in a pandas Dataframe
        self.odo_csv = pd.read_csv(self.odo)

        self.pose_x = np.array(self.odo_csv["pose_x"])
        self.pose_y = np.array(self.odo_csv["pose_y"])
        self.pose_theta = np.array(self.odo_csv["pose_theta"])

        """orient_x = np.array(odo_csv["pose.orientation.x"])
        orient_y = np.array(odo_csv["pose.orientation.y"])
        orient_z = np.array(odo_csv["pose.orientation.z"])
        orient_w = np.array(odo_csv["pose.orientation.w"])

        siny_cosp = 2 * np.add(np.multiply(orient_w, orient_z), np.multiply(orient_x, orient_y))
        cosy_cosp = 1 - 2 * np.add(np.multiply(orient_y, orient_y), np.multiply(orient_z, orient_z))
        angles_yaw = np.arctan2(siny_cosp, cosy_cosp) # yaw (z-axis rotation)
        """

        self.w1 = np.array(self.odo_csv["w1"])
        self.w2 = np.array(self.odo_csv["w2"])
        self.w3 = np.array(self.odo_csv["w3"])
        self.w4 = np.array(self.odo_csv["w4"])

        self.t_s = np.add(1000000000 * np.array(self.odo_csv["header.stamp.secs"]), np.array(self.odo_csv["header.stamp.nsecs"])) / 1000000000

        # compute robot velocities
        self.vx = np.add(np.add(np.add(self.w1, self.w2), self.w3), self.w4) * self.r / 4.0 / 60.0 / self.gear_ratio
        self.vy = np.add(np.add(np.add(-1 * self.w1, self.w2), self.w3), -1 * self.w4) * self.r / 4.0 / 60.0 / self.gear_ratio
        self.omega = np.add(np.add(np.add(-1 * self.w1, self.w2), -1 * self.w3), self.w4) * (self.r / 4.0 / (self.l + self.w)) / 60.0 / self.gear_ratio


#odometry computations

    def cycle(self):
        #print (self.omega)
        theta_hat = np.zeros(len(self.t_s))
        current_x = np.zeros(len(self.t_s))
        current_y = np.zeros(len(self.t_s))
        current_theta = np.zeros(len(self.t_s))
        delta_t = np.zeros(len(self.t_s))

        for t in range(1, len(self.t_s)):
            delta_t[t] = self.t_s[t] - self.t_s[t-1]
            theta_hat[t] = current_theta[t-1] + self.omega[t] * delta_t[t] / 2
            current_x[t] = current_x[t-1] + (self.vx[t] * np.cos(theta_hat[t]) - self.vy[t] * np.sin(theta_hat[t])) * delta_t[t]
            current_y[t] = current_y[t-1] + (self.vx[t] * np.sin(theta_hat[t]) + self.vy[t] * np.cos(theta_hat[t])) * delta_t[t]
            current_theta[t] = current_theta[t-1] + self.omega[t] * delta_t[t]
        return (current_x, current_y, current_theta)



def main():

    estimator = Estimator('../../../bags/dataset_bag3.bag')
    (x, y, theta) = estimator.cycle()

    estimator.t_s = estimator.t_s - estimator.t_s[0]; # time scaling factor

    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose x")
    plt.plot(estimator.t_s, x)
    plt.show()

if __name__ == "__main__":
    main()
