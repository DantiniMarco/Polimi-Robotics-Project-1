import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class Estimator:

    def __init__(self, csv, lw, rg):

        # robot standard values
        self.lw = lw
        self.rg = rg

        # time deltas
        self.t_s = np.add(np.array(csv["header.stamp.secs"]), np.array(csv["header.stamp.nsecs"]) / 1000000000)

        # pose read from /robot/pose
        self.pose_x = np.array(csv["pose_x"])
        self.pose_y = np.array(csv["pose_y"])
        self.pose_theta = np.array(csv["pose_theta"])

        # wheels speeds read from /wheel_states
        self.w1 = np.array(csv["w1"])
        self.w2 = np.array(csv["w2"])
        self.w3 = np.array(csv["w3"])
        self.w4 = np.array(csv["w4"])

        # computed odometry initializations
        self.current_x = np.zeros(len(self.t_s))
        self.current_y = np.zeros(len(self.t_s))
        self.current_theta = np.zeros(len(self.t_s))

        # compute robot velocities
        self.vx = np.add(np.add(np.add(self.w1, self.w2), self.w3), self.w4) * self.rg / 60.0
        self.vy = np.add(np.add(np.add(-1 * self.w1, self.w2), self.w3), -1 * self.w4) * self.rg / 60.0
        self.omega = np.add(np.add(np.add(-1 * self.w1, self.w2), -1 * self.w3), self.w4) * (self.rg * self.lw / 60.0)


    #odometry computations
    def cycle(self):
        delta_t = np.zeros(len(self.t_s))
        theta_hat = np.zeros(len(self.t_s))
        for t in range(1, len(self.t_s)):
            delta_t[t] = self.t_s[t] - self.t_s[t-1]
            theta_hat[t] = self.current_theta[t-1] + self.omega[t] * delta_t[t] / 2
            self.current_x[t] = self.current_x[t-1] + (self.vx[t] * np.cos(theta_hat[t]) - self.vy[t] * np.sin(theta_hat[t])) * delta_t[t]
            self.current_y[t] = self.current_y[t-1] + (self.vx[t] * np.sin(theta_hat[t]) + self.vy[t] * np.cos(theta_hat[t])) * delta_t[t]
            self.current_theta[t] = self.current_theta[t-1] + self.omega[t] * delta_t[t]

    # compute error value from difference of true pose and computed odometry
    def error_squared(self):
        # using x pose because it seems to have the most accurate comparison values
        error = np.add(self.pose_x, -1 * self.current_x)
        error **= 2
        e_sum = np.sum(error)
        return e_sum


def main():
    # read bag file (relative directory)
    # read dataset from recorded bag (data from bag3)
    b = bagreader('../../../bags/dataset_bag3.bag')
    # read topic with all data
    odo = b.message_by_topic('/recorder')
    # memorize data read from bag in a pandas Dataframe
    csv = pd.read_csv(odo)

    # standard values given for the project assignment
    lw = 2.7 #lw = l + w
    rg = 0.0035 #rg = r / 4 / gear_ratio

    #iteration over multiple values to check the best value for the parameters
    """ # closer ranges for more precise computation
    iter_rg = np.linspace(0.0032, 0.0034, 20)
    iter_lw = np.linspace(2.682, 2.684, 20)
    error = np.zeros(20)
    error_2 = np.zeros(20)
    """
    iter_rg = np.linspace(0.003, 0.004, 100)
    iter_lw = np.linspace(2.6, 2.8, 100)
    error = np.zeros(100)
    error_2 = np.zeros(100)

    print("Given fixed lw = " + str(lw) + ", computing optimal rg value")
    k = 0
    for i in iter_rg:
        estimator = Estimator(csv, lw, i)
        estimator.cycle()
        error[k] = estimator.error_squared()
        k = k+1
    print("Minimum error for lw = " + str(lw) + ", rg = " + str(iter_rg[np.argmin(error)]) + ": error = " + str(np.min(error)))


    print("Given fixed rg = " + str(rg) + ", computing optimal lw value")
    k = 0
    for j in iter_lw:
        estimator = Estimator(csv, j, rg)
        estimator.cycle()
        error_2[k] = estimator.error_squared()
        k = k+1

    print("Minimum error for rg = " + str(rg) + ", lw = " + str(iter_lw[np.argmin(error_2)]) + ": error = " + str(np.min(error_2)))

    # given rg = 0.003335 , error = 43.768 is minimized
    plt.figure(1)
    plt.title("fixing lw, error function of rg value")
    plt.ylabel("squared error value")
    plt.xlabel("rg value")
    plt.plot(iter_rg, error)

    # given lw = 2.6829, error = 54.37 is minimized
    plt.figure(2)
    plt.title("fixing rg, error function of lw value")
    plt.ylabel("squared error value")
    plt.xlabel("lw value")
    plt.plot(iter_lw, error_2)

    # optimal values found for lw, rg
    estimator = Estimator(csv, 2.6829, 0.003335)
    estimator.cycle()
    print("Given optimal parameters (lw = 2.6829, rg = 0.003335), error = " + str(estimator.error_squared()))

    estimator.t_s = estimator.t_s - estimator.t_s[0]; # time scaling factor

    plt.figure(3)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose x")
    plt.plot(estimator.t_s, estimator.current_x, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_x, label = "given pose x")
    plt.legend()

    plt.figure(4)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose y")
    plt.plot(estimator.t_s, estimator.current_y, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_y, label = "given pose y")
    plt.legend()

    plt.figure(5)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose theta")
    plt.plot(estimator.t_s, estimator.current_theta, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_theta, label = "given pose theta")
    plt.legend()

    plt.show()

    """
    Simulation results:
    l = 0.2 m
    w = 0.1727 m
    r = 0.0667 m
    gear_ratio = 5
    """


if __name__ == "__main__":
    """
    - Wheel radius (r): 0.07 m
    - Wheel position along x (l): 0.200 m
    - Wheel position along y (w): 0.169 m
    - Gear ratio (gear_ratio): 5
    """
    main()
