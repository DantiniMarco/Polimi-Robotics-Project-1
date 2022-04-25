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
        # read bag file (relative directory)
        self.csv = csv
       
        self.t_s = np.add(1000000000 * np.array(self.csv["header.stamp.secs"]), np.array(self.csv["header.stamp.nsecs"])) / 1000000000

        self.pose_x = np.array(self.csv["pose_x"])
        self.pose_y = np.array(self.csv["pose_y"])
        self.pose_theta = np.array(self.csv["pose_theta"])


        self.theta_hat = np.zeros(len(self.t_s))
        self.current_x = np.zeros(len(self.t_s))
        self.current_y = np.zeros(len(self.t_s))
        self.current_theta = np.zeros(len(self.t_s))
        self.delta_t = np.zeros(len(self.t_s))


        """orient_x = np.array(odo_csv["pose.orientation.x"])
        orient_y = np.array(odo_csv["pose.orientation.y"])
        orient_z = np.array(odo_csv["pose.orientation.z"])
        orient_w = np.array(odo_csv["pose.orientation.w"])

        siny_cosp = 2 * np.add(np.multiply(orient_w, orient_z), np.multiply(orient_x, orient_y))
        cosy_cosp = 1 - 2 * np.add(np.multiply(orient_y, orient_y), np.multiply(orient_z, orient_z))
        angles_yaw = np.arctan2(siny_cosp, cosy_cosp) # yaw (z-axis rotation)
        """

        self.w1 = np.array(self.csv["w1"])
        self.w2 = np.array(self.csv["w2"])
        self.w3 = np.array(self.csv["w3"])
        self.w4 = np.array(self.csv["w4"])

       

        # compute robot velocities
        self.vx = np.add(np.add(np.add(self.w1, self.w2), self.w3), self.w4) * self.rg / 60.0 
        self.vy = np.add(np.add(np.add(-1 * self.w1, self.w2), self.w3), -1 * self.w4) * self.rg / 60.0
        self.omega = np.add(np.add(np.add(-1 * self.w1, self.w2), -1 * self.w3), self.w4) * (self.rg * self.lw / 60.0)


#odometry computations

    def cycle(self):
        #print (self.omega)
        for t in range(1, len(self.t_s)):
            self.delta_t[t] = self.t_s[t] - self.t_s[t-1]
            self.theta_hat[t] = self.current_theta[t-1] + self.omega[t] * self.delta_t[t] / 2
            self.current_x[t] = self.current_x[t-1] + (self.vx[t] * np.cos(self.theta_hat[t]) - self.vy[t] * np.sin(self.theta_hat[t])) * self.delta_t[t]
            self.current_y[t] = self.current_y[t-1] + (self.vx[t] * np.sin(self.theta_hat[t]) + self.vy[t] * np.cos(self.theta_hat[t])) * self.delta_t[t]
            self.current_theta[t] = self.current_theta[t-1] + self.omega[t] * self.delta_t[t]

    def error_squared(self):
        error = np.add(self.pose_x, -1 * self.current_x)
        error **= 2
        e_sum = np.sum(error)
        return e_sum






def main():

    b = bagreader('../../../bags/dataset_bag3.bag')

    # read topic
    odo = b.message_by_topic('/recorder')
    # memorize data read from bag in a pandas Dataframe
    csv = pd.read_csv(odo)
    

    lw = 2.71
    rg = 0.0035

    """
    iter_rg = np.linspace(0.0032, 0.0034, 20)
    iter_lw = np.linspace(2.682, 2.684, 20)

    error = np.zeros(20)
    error_2 = np.zeros(20)
    k = 0

    for i in iter_rg:
        estimator = Estimator(csv, lw, i)
        estimator.cycle()
        error[k] = estimator.error_squared()
        k = k+1
    k = 0
    print("Min: " + str(np.min(error)))
    for j in iter_lw:
        estimator = Estimator(csv, j, rg)
        estimator.cycle()
        error_2[k] = estimator.error_squared()
        k = k+1
    print("Min: " + str(np.min(error_2)))
    """




    estimator = Estimator(csv, 2.6829, 0.003335)
    estimator.cycle()



    estimator.t_s = estimator.t_s - estimator.t_s[0]; # time scaling factor

    
    plt.title("robot odometry")
    plt.xlabel("time [s]")
    plt.ylabel("robot pose x")
    plt.plot(estimator.t_s, estimator.current_y)
    plt.plot(estimator.t_s, estimator.pose_y)
    plt.show()


"""
    #min error = 0.003335 (43.768)
    plt.title("error on rg")
    plt.plot(iter_rg, error)
    plt.show()

    #min error = 2.6829 (54.37)
    plt.title("error on lw")
    plt.plot(iter_lw, error_2)
    plt.show()
    """



if __name__ == "__main__":
    main()
