#include "ros/ros.h"
#include <math.h>

// Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped

using namespace std;

class Odometry {
public:
private:
    double l = 0.2, w = 0.169, r = 0.07;
    double omega, vx, vy;

    /*
      Front left wheel = 1
      Front right wheel = 2
      Rear right wheel = 3
      Rear left wheel = 4
    */
    double w1, w2, w3, w4; // input in [rad/min]
    float t_s; //time of sampling

    /*
     * omega: angular velocity
     * vx: linear velocity along x [m/s]
     * vy: linear velocity along y [m/s]
     * w1,w2,w3,w4: angular velocities of the wheels [rad/s]
     * l,w = robot dimension
     * r = wheel radius
     */
    void computeOmega() {

        w1 = (-(l+w)/r) * omega + vbx/r - vby/r;
        w2 = ((l+w)/r) * omega + vbx/r + vby/r;
        w3 = ((l+w)/r) * omega + vbx/r - vby/r;
        w4 = (-(l+w)/r) * omega + vbx/r + vby/r;

    }
    /*
     * La formula per il calcolo di vbx,vby e omega è riadattata alle nostre convenzioni consierando
     * che in quel PDF trovato da Simone scambiava la ruota 3 con la ruota 4
     */
    void computeVelocities() {

        vbx = (w1 + w2 + w3 + w4) * (r/4.0) * 60.0;
        vby = (-w1 + w2 - w3 + w4) * (r/4.0) * 60.0;
        omega = (-w1 + w2 + w3 - w4) * (r/4.0 / (l+w)) * 60.0;

    }

    void integrations() {
        if (integration_method == 1){ //Euler
            time_difference = msg->header.stamp - current_time;
            t_s = time_difference.toSec();

            //formulas
            current_x = current_x + msg->twist.linear.x * t_s * cos(current_theta);
            current_y = current_y + msg->twist.linear.x * t_s * sin(current_theta);
            current_theta = current_theta + msg->twist.angular.z * t_s;

            current_quaternion.setRPY(0,0,current_theta);

            custom_odometry.odom.header.stamp = msg->header.stamp;
            custom_odometry.odom.pose.pose.position.x = current_x;
            custom_odometry.odom.pose.pose.position.y = current_y;
            custom_odometry.odom.pose.pose.orientation.z = current_quaternion[2];
            custom_odometry.odom.pose.pose.orientation.w = current_quaternion[3];
            custom_odometry.method.data = "euler";

            current_time = msg->header.stamp;

        } else if (integration_method == 2){ //Runge-Kutta
            time_difference = msg->header.stamp - current_time;
            t_s = time_difference.toSec();

            //formulas
            current_x = current_x + msg->twist.linear.x * t_s * cos(current_theta + msg->twist.angular.z * t_s /2);
            current_y = current_y + msg->twist.linear.x * t_s * sin(current_theta + msg->twist.angular.z * t_s /2);
            current_theta = current_theta + msg->twist.angular.z * t_s;


            current_quaternion.setRPY(0,0,current_theta);

            custom_odometry.odom.header.stamp = msg->header.stamp;
            custom_odometry.odom.pose.pose.position.x = current_x;
            custom_odometry.odom.pose.pose.position.y = current_y;
            custom_odometry.odom.pose.pose.orientation.z = current_quaternion[2];
            custom_odometry.odom.pose.pose.orientation.w = current_quaternion[3];
            custom_odometry.method.data = "rk";

            current_time = msg->header.stamp;
        }
    }

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_computation");

    Odometry odometry;

    ros::spin(); // needed

    return 0;

}
