#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h" // wheel_states messages
#include "geometry_msgs/TwistStamped.h" // published messages
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include "robo/parametersConfig.h" // parameters file config
#include "robo/odom.h" // odometry message
#include "robo/speeds.h" // speeds message
#include "robo/set_odometry.h" // odometry service

#include <cmath>

using namespace robo;

enum integration_methods {EULER, RUNGE_KUTTA};

class Odometry {
private:
    const double l = 0.2, w = 0.169, r = 0.07, gear_ratio = 5.0, tick_count = 42.0;

    double omega, vx, vy; // velocities computed from wheel speeds
    double new_x, new_y, new_theta; // new odometry set by service


    // x, y, theta values set by parameters, can be also dynamically reconfigured
    double current_x, current_y, current_theta;
    integration_methods integration_method; // switch between euler and runge-kutta
    bool reset; // flag that indicates whether the set service has been called

    /*
    Front left wheel = 1
    Front right wheel = 2
    Rear left wheel = 3
    Rear right wheel = 4
    */
    double w1, w2, w3, w4; // input in [rad/min]

    //messages to be published
    geometry_msgs::TwistStamped velocities; // v and w velocities computed from wheel speeds
    odom custom_odometry; //computed odometry

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
    geometry_msgs::TransformStamped transformStamped;

    //dynamic reconfiguration
    dynamic_reconfigure::Server<parametersConfig> server;
    dynamic_reconfigure::Server<parametersConfig>::CallbackType callback_f;

    void wheel_state_callback(const sensor_msgs::JointStateConstPtr& msg);
    void callback_publisher_timer(const ros::TimerEvent&);
    bool callback_set_odometry(set_odometry::Request &request, set_odometry::Response &response);
    void callback_dynamic_reconfigure(parametersConfig &config, uint32_t level);

    void computeOmega();
    void computeVelocities();
    void integrations(const sensor_msgs::JointStateConstPtr& msg);

public:
    Odometry();
};

#endif
