#include "odometry_project/Control.h"

Control::Control() {

	// subscriber to the computed robot velocities and set the callbacks
    sub = node.subscribe("/cmd_vel", 1000, &Control::wheel_velocities_callback, this);
    // publisher for the computed wheels velocities
    pub_omega = node.advertise<odometry_project::wheels_rpm>("/wheels_rpm", 1);
    // creates a timer callback that updates the odometry and parameters each time the timer expires
    timer = node.createTimer(ros::Duration(0.01), &Control::callback_publisher_timer, this);

}

/**
  * takes the robot linear speeds and calculates the wheels speeds
  */
void Control::wheel_velocities_callback(const geometry_msgs::TwistStampedConstPtr& msg) {

    current_time = msg->header.stamp;
    vx = msg-> twist.linear.x;
    vy = msg-> twist.linear.y;
    omega = msg-> twist.angular.z;

    computeOmega();

    omega_msg.header.stamp = current_time;
    omega_msg.header.seq += 1 ;

    omega_msg.rpm_fl = w1;
    omega_msg.rpm_fr = w2;
    omega_msg.rpm_rr = w4;
    omega_msg.rpm_rl = w3;

}

/**
 *  callbacks that is called every time the timer event expires.
 *  Publishes messages containing computed data from bags input
 */
void Control::callback_publisher_timer(const ros::TimerEvent& ev) {
    if ((latest_sent_time - current_time).toSec() != 0) {
        latest_sent_time = current_time;

        //publish omega velocities of the 4 wheels
        pub_omega.publish(omega_msg);
    }
}

/*
* omega: angular velocity of the robot -> YAW orientation in [RPM = revolutions per minute]
* vx: linear velocity along x [m/s]
* vy: linear velocity along y [m/s]
* w1,w2,w3,w4: angular velocities of the wheels [rad/s]
*/
void Control::computeOmega() {

    double k = 30.0 * gear_ratio / M_PI;
    w1 = ((-(l + w) / r) * omega + vx / r - vy / r) * k;
    w2 = (((l + w) / r) * omega + vx / r + vy / r) * k;
    w3 = ((-(l + w) / r) * omega + vx / r + vy / r) * k;
    w4 = (((l + w) / r) * omega + vx / r - vy / r) * k;

}

int main(int argc, char** argv){
	ros::init(argc, argv, "control_computation");

    Control control; // calls constructor

    ros::spin(); // needed

    return 0;

}
