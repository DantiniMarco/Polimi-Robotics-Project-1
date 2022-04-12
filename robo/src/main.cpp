#include "ros/ros.h"

#include "sensor_msgs/JointState.h" // wheel_states messages
#include "geometry_msgs/TwistStamped.h" // published messages

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

    geometry_msgs::TwistStamped velocities; // message to be published

    float t_s; //time of sampling

    Odometry() {
        ros::NodeHandle node;
        ros::Subscriber sub = node.subscribe("wheel_states", 1000, &Odometry::wheel_state_callback);

        ros::Publisher pub = node.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

        ros::Timer timer = n.createTimer(ros::Duration(0.01), &timer_callback);

        dynamic_reconfigure::Server<robo::parametersConfig> dynServer;
        dynamic_reconfigure::Server<robo::parametersConfig>::CallbackType f;
        f = boost::bind(&param_callback, &mode, &fmt, _1, _2);
        dynServer.setCallback(f);

        // robot/pose
        //msg->pose.position.//x,y,z;
        //msg->pose.orientation.//x,y,z,w

    }

    void Odometry::wheel_state_callback(const robo::sensor_msgs::JointStateConstPtr& msg) {
        w1 = msg->velocity[0];
        w2 = msg->velocity[1];
        w3 = msg->velocity[2];
        w4 = msg->velocity[3];

        computeVelocities();

    }

    void publisher() {
        //Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped
    }

    //callback to copy the filtered twist to the message in order for the publisher to publish it
    void callback_publisher_timer(const ros::TimerEvent&) {
        if ((latest_sent_time-current_time).toSec() != 0) {
            latest_sent_time = current_time;
            pub.publish(custom_odometry);
        }
    }

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

        vx = (w1 + w2 + w3 + w4) * (r/4.0) * 60.0;
        vy = (-w1 + w2 - w3 + w4) * (r/4.0) * 60.0;
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
