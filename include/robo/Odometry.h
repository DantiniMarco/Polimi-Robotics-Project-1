#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h" // wheel_states messages
#include "geometry_msgs/TwistStamped.h" // published messages
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <Odometry/parametersConfig.h>
#include <math.h>

// Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped

class Odometry {
private:
    const double l = 0.2, w = 0.169, r = 0.07, gear_ratio = 5.0;
    double omega, vx, vy, new_x, new_y, new_theta;
    int integration_method;

    /*
      Front left wheel = 1
      Front right wheel = 2
      Rear right wheel = 3
      Rear left wheel = 4
    */
    double w1, w2, w3, w4; // input in [rad/min]

    geometry_msgs::TwistStamped velocities; // v and w velocities computed from wheel speeds to be published

    nav_msgs::Odometry odometry_msg; //computed odometry to be published 

    geometry_msgs::TransformStamped transformStamped;

    float t_s; //time of sampling
    ros::Time current_time = ros::Time(0);
    ros::Time latest_sent_time = ros::Time(0);

    ros::Subscriber sub;
    ros::Publisher pub_speeds;
    ros::Publisher pub_odom;
    ros::NodeHandle node;
    ros::Timer timer;
    ros::ServiceServer serviceSet;

    tf2::Quaternion current_quaternion;
    tf2_ros::TransformBroadcaster br;

    dynamic_reconfigure::Server<Odometry::parametersConfig> server;
    dynamic_reconfigure::Server<Odometry::parametersConfig>::CallbackType callback_f;



public:
    Odometry();

    void wheel_state_callback(const sensor_msgs::JointStateConstPtr& msg);
    void publisher();
    void callback_publisher_timer(const ros::TimerEvent&);
    void computeOmega();
    void computeVelocities();
    void integrations();

};

#endif
