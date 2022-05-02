import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

class Estimator:

    def __init__(self, csv, lw, r, n):

        # robot standard values
        self.lw = lw
        self.r = r
        self.n = n
        gear_ratio = 5

        # time deltas
        self.t_s = np.add(np.array(csv["header.stamp.secs"]), np.array(csv["header.stamp.nsecs"]) / 1000000000)

        # pose read from /robot/pose
        self.pose_x = np.array(csv["pose_x"])
        self.pose_y = np.array(csv["pose_y"])
        self.pose_theta = np.array(csv["pose_theta"])

        # wheels speeds read from /wheel_states
        t1 = np.array(csv["ticks1"])
        t2 = np.array(csv["ticks2"])
        t3 = np.array(csv["ticks3"])
        t4 = np.array(csv["ticks4"])

        w1 = np.zeros(len(t1));
        w2 = np.zeros(len(t2));
        w3 = np.zeros(len(t3));
        w4 = np.zeros(len(t4));

        for i in range(1, len(t1)):
            s = 2 * math.pi / self.n / (self.t_s[i] - self.t_s[i-1])
            w1[i] = (t1[i] - t1[i-1]) * s
            w2[i] = (t2[i] - t2[i-1]) * s
            w3[i] = (t3[i] - t3[i-1]) * s
            w4[i] = (t4[i] - t4[i-1]) * s


        # computed odometry initializations
        self.current_x = np.zeros(len(self.t_s))
        self.current_y = np.zeros(len(self.t_s))
        self.current_theta = np.zeros(len(self.t_s))

        # compute robot velocities
        self.vx = np.add(np.add(np.add(w1, w2), w3), w4) * self.r / 4 / gear_ratio
        self.vy = np.add(np.add(np.add(-1 * w1, w2), w3), -1 * w4) * self.r / 4 / gear_ratio
        self.omega = np.add(np.add(np.add(-1 * w1, w2), -1 * w3), w4) * self.r / self.lw / 4 / gear_ratio


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
        error_1 = np.add(self.pose_x, -1 * self.current_x)
        error_1 **= 2
        error_2 = np.add(self.pose_y, -1 * self.current_y)
        error_2 **= 2
        e_sum = np.sum(error_1) + np.sum(error_2)
        return e_sum


def main():
    # read bag file (relative directory)
    # read dataset from recorded bag (data from bag3)
    b = bagreader('../../../bags/dataset_rec.bag')
    # read topic with all data
    odo = b.message_by_topic('/recorder')
    # memorize data read from bag in a pandas Dataframe
    csv = pd.read_csv(odo)

    # standard values given for the project assignment
    lw = 0.369 # lw = l + w
    r = 0.07 # rg = r / 4 / gear_ratio
    N = 42 # TODO: change N

    #iteration over multiple values to check the best value for the parameters

    iter_r = np.linspace(0.076, 0.08, 100)
    iter_lw = np.linspace(0.36, 0.38, 100)
    iter_n = np.linspace(40, 44, 5)
    error_r = np.zeros(100)
    error_lw = np.zeros(100)
    error_n = np.zeros(5)


    k = 0
    for i in iter_r:
        estimator = Estimator(csv, lw, i, N)
        estimator.cycle()
        error_r[k] = estimator.error_squared()
        k = k+1

    min_r = iter_r[np.argmin(error_r)]
    print("Minimum error for lw = " + str(lw) + ", r = " + str(min_r) + ", n = " + str(N) + ": error = " + str(np.min(error_r)))

    k = 0
    for h in iter_n:
        estimator = Estimator(csv, lw, min_r, h)
        estimator.cycle()
        error_n[k] = estimator.error_squared()
        k = k+1

    min_n = iter_n[np.argmin(error_n)]
    print("Minimum error for r = " + str(min_r) + ", lw = " + str(lw) + ", n = " + str(min_n) + ": error = " + str(np.min(error_n)))

    k = 0
    for j in iter_lw:
        estimator = Estimator(csv, j, min_r, min_n)
        estimator.cycle()
        error_lw[k] = estimator.error_squared()
        k = k+1

    min_lw = iter_lw[np.argmin(error_lw)]
    print("Minimum error for r = " + str(min_r) + ", lw = " + str(min_lw) + ", n = " + str(min_n) + ": error = " + str(np.min(error_lw)))


    # plots the error as function of the variable rg value, while lw is fixed
    plt.figure(1)
    plt.title("fixing lw and n, error function of r value")
    plt.ylabel("squared error value")
    plt.xlabel("r value")
    plt.plot(iter_r, error_r)
    plt.grid()

    # plots the error as function of the variable lw value, while rg is fixed
    plt.figure(2)
    plt.title("fixing r and n, error function of lw value")
    plt.ylabel("squared error value")
    plt.xlabel("lw value")
    plt.plot(iter_lw, error_lw)
    plt.grid()

    # plots the error as function of the variable lw value, while rg is fixed
    plt.figure(3)
    plt.title("fixing r and lw, error function of n value")
    plt.ylabel("squared error value")
    plt.xlabel("n value")
    plt.plot(iter_n, error_n)
    plt.grid()

    # optimal values found for lw, r
    estimator = Estimator(csv, min_lw, min_r, min_n)
    estimator.cycle()

    print("Given optimal parameters , error = " + str(estimator.error_squared()))

    estimator.t_s = estimator.t_s - estimator.t_s[0]; # time scaling factor

    plt.figure(4)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose x")
    plt.plot(estimator.t_s, estimator.current_x, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_x, label = "given pose x")
    plt.legend()

    plt.figure(5)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose y")
    plt.plot(estimator.t_s, estimator.current_y, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_y, label = "given pose y")
    plt.legend()

    plt.figure(6)
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose theta")
    plt.plot(estimator.t_s, estimator.current_theta, label = "computed odometry")
    plt.plot(estimator.t_s, estimator.pose_theta, label = "given pose theta")
    plt.legend()

    plt.show()

    """
    Simulation results:
    l = 0.2 m (fixed for simplicity)
    w = 0.16648 m
    r = 0.0704 m
    gear_ratio = 5 (fixed)
    """


if __name__ == "__main__":
    """ # initial values
    - Wheel radius (r): 0.07 m
    - Wheel position along x (l): 0.200 m
    - Wheel position along y (w): 0.169 m
    - Gear ratio (gear_ratio): 5
    - N = 42
    """
    main()
