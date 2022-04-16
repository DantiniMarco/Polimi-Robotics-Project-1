#include "robo/Odometry.h"

Odometry::Odometry() {

    // retrieves parameter values from the launch file configuration
    node.getParam("/pose_x", current_x);
    node.getParam("/pose_y", current_y);
    node.getParam("/pose_theta", current_theta);

    // subscriber to the wheel velocities and set the callbacks
    sub = node.subscribe("/wheel_states", 1000, &Odometry::wheel_state_callback, this);

    // publisher of the robots speeds
    pub_speeds = node.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    // publisher of the robot pose calculated with odometry formulas
    pub_odom = node.advertise<robo::odom>("/odom", 1);

    // creates a timer callback that updates the odometry and parameters each time the timer expires
    timer = node.createTimer(ros::Duration(0.01), &Odometry::callback_publisher_timer, this);

    // dynamic reconfiguration of the paramters for the robot pose
    callback_f = boost::bind(&Odometry::callback_dynamic_reconfigure, this, _1, _2);
    server.setCallback(callback_f);

    // service that allows to change the robot pose at runtime
    serviceSet = node.advertiseService<set_odometry::Request, set_odometry::Response>(
        "set_odometry",
        boost::bind(&Odometry::callback_set_odometry, this, _1, _2)
    );

    reset = false;
}

/**
 *  Callback of the subscriber to the wheel_states topic. Computes velocity and robot odometry
 */
void Odometry::wheel_state_callback(const sensor_msgs::JointStateConstPtr& msg) {

    if (current_time == ros::Time(0)) { // start of execution
        current_x = 0.0;
        current_y = 0.0;
        current_theta = 0.0;

    } else if (reset) { // service was called
        reset = false;

        current_x = new_x;
        current_y = new_y;
        current_theta = new_theta;
    } else { // computation

        w1 = msg->velocity[0]; // front left
        w2 = msg->velocity[1]; // front right
        w3 = msg->velocity[2]; // rear left
        w4 = msg->velocity[3]; // rear right

        computeVelocities(); // calculates robot velocity

        velocities.twist.linear.x = vx;
        velocities.twist.linear.y = vy;
        velocities.twist.angular.z = omega;
        velocities.header.stamp = msg->header.stamp;

        integrations(msg); // integrate velocities to find robot pose
    }

    current_time = msg->header.stamp;

    // odometry message
    custom_odometry.odom.header.stamp = msg->header.stamp;
    custom_odometry.odom.pose.pose.position.x = current_x;
    custom_odometry.odom.pose.pose.position.y = current_y;
    current_quaternion.setRPY(0, 0, current_theta);
    custom_odometry.odom.pose.pose.orientation.x = current_quaternion[0];
    custom_odometry.odom.pose.pose.orientation.y = current_quaternion[1];
    custom_odometry.odom.pose.pose.orientation.z = current_quaternion[2];
    custom_odometry.odom.pose.pose.orientation.w = current_quaternion[3];

    // transformation: set header
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    // transformation: set x,y,z
    transformStamped.transform.translation.x = current_x;
    transformStamped.transform.translation.y = current_y;
    transformStamped.transform.translation.z = 0.0;
    // transformation: set theta with quaternion
    transformStamped.transform.rotation.x = current_quaternion.x();
    transformStamped.transform.rotation.y = current_quaternion.y();
    transformStamped.transform.rotation.z = current_quaternion.z();
    transformStamped.transform.rotation.w = current_quaternion.w();

    // broadcast transform
    br.sendTransform(transformStamped);
}

/**
 *  compute odometry from velocity with two different integration methods
*/
void Odometry::integrations(const sensor_msgs::JointStateConstPtr& msg) {
    double vel = sqrt(vx*vx + vy*vy);

    ros::Duration time_difference = msg->header.stamp - current_time;
    double t_s = time_difference.toSec(); //time of sampling

    if (integration_method == EULER){ // Euler
        // formulas
        current_x = current_x + vel * t_s * cos(current_theta);
        current_y = current_y + vel * t_s * sin(current_theta);
        current_theta = current_theta + omega * t_s;

        custom_odometry.method.data = "euler";

    } else if (integration_method == RUNGE_KUTTA){ // Runge-Kutta
        // formulas
        current_x = current_x + vel * t_s * cos(current_theta + omega * t_s / 2.0);
        current_y = current_y + vel * t_s * sin(current_theta + omega * t_s / 2.0);
        current_theta = current_theta + omega * t_s;

        custom_odometry.method.data = "runge-kutta";
    }

}

/**
 *  callbacks that is called every time the timer event expires.
 *  Publishes messages containing computed data from bags input
 */
void Odometry::callback_publisher_timer(const ros::TimerEvent& ev) {
    if ((latest_sent_time - current_time).toSec() != 0) {
        latest_sent_time = current_time;

        //Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped
        pub_speeds.publish(velocities);

        // ROS parameter for initial pose (x,y,θ) published as custom odometry message
        pub_odom.publish(custom_odometry);
    }
}

/**
 *  Service callback that changes the robot pose (x, y, theta parameters)asynchronously
 */
bool Odometry::callback_set_odometry(set_odometry::Request &request, set_odometry::Response &response) {
    new_x = request.x;
    new_y = request.y;
    new_theta = request.theta;
    ROS_INFO("Request to set odometry received. New values: x = %lf, y = %lf, theta = %lf", new_x, new_y, new_theta);
    reset = true;
    return true;
}

/**
 *  Dynamic reconfiguration callback that changes the integration method at runtime
 */
void Odometry::callback_dynamic_reconfigure(parametersConfig &config, uint32_t level) {
    std::string method;
    if (config.integration_method == 1) {
        integration_method = EULER;
        method = "Euler";
    } else if (config.integration_method == 2) {
        integration_method = RUNGE_KUTTA;
        method = "Runge-Kutta";
    }

    ROS_INFO("Request to dynamically reconfigure the integration method received: now using %s", method.c_str());
}

/**
 *  calculation of the robot velocities from the speeds of the wheels
*/
void Odometry::computeVelocities() {
    vx = (w1 + w2 + w3 + w4) * (r / 4.0) / 60.0 / gear_ratio;
    vy = (-w1 + w2 + w3 - w4) * (r / 4.0) / 60.0 / gear_ratio;
    omega = (-w1 + w2 - w3 + w4) * (r / 4.0 / (l + w)) / 60.0 / gear_ratio;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_computation");

    Odometry odometry; // calls constructor

    ros::spin(); // needed

    return 0;

}
