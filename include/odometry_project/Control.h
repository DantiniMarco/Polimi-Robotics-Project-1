#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h" // wheel_states messages
#include "geometry_msgs/TwistStamped.h" // published messages
#include <cmath>

#include "odometry_project/wheels_rpm.h"

using namespace odometry_project;

class Control{
private:
	// optimal values found with the Python script
    const double l = 0.2, w = 0.1579, r = 0.07579, gear_ratio = 5.0, tick_count = 42.0;

    ros::Time current_time = ros::Time(0);
	ros::Time latest_sent_time = ros::Time(0);

    ros::Publisher pub_omega; //publisher

    ros::Subscriber sub; //subscriber
    ros::NodeHandle node;
    ros::Timer timer;

    wheels_rpm omega_msg; // message containing the speeds of the wheels

   	double vx, vy, omega, w1, w2, w3, w4; // robot speed and wheels speeds

	void computeOmega();
	void callback_publisher_timer(const ros::TimerEvent& ev);
    void wheel_velocities_callback(const geometry_msgs::TwistStampedConstPtr& msg);


public:
	Control();

};

#endif
