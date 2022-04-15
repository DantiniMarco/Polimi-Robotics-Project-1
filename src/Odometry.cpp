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

    serviceSet = node.advertiseService<set_odometry::Request, set_odometry::Response>(
        "set_odometry",
        boost::bind(&Odometry::callback_set_odometry, &new_x, &new_y, &new_theta, _1, _2, _3, _4)
    );

    reset = false;

}

void Odometry::wheel_state_callback(const sensor_msgs::JointStateConstPtr& msg) {
    if (current_time == ros::Time(0)) {
        current_x = 0.0;
        current_y = 0.0;
        current_theta = 0.0;
        current_time = msg->header.stamp;
    } else if (reset) {
        reset = false;

        current_x = new_x;
        current_y = new_y;
        current_theta = new_theta;
    } else {

        w1 = msg->velocity[0];
        w2 = msg->velocity[1];
        w3 = msg->velocity[2];
        w4 = msg->velocity[3];

        computeVelocities();

        velocities.twist.linear.x = vx;
        velocities.twist.linear.y = vy;
        velocities.twist.angular.z = omega;

        integrations(msg);
    }

    custom_odometry.odom.header.stamp = msg->header.stamp;
    custom_odometry.odom.pose.pose.position.x = current_x;
    custom_odometry.odom.pose.pose.position.y = current_y;
    current_quaternion.setRPY(0, 0, current_theta);
    custom_odometry.odom.pose.pose.orientation.x = current_quaternion[0];
    custom_odometry.odom.pose.pose.orientation.y = current_quaternion[1];
    custom_odometry.odom.pose.pose.orientation.z = current_quaternion[2];
    custom_odometry.odom.pose.pose.orientation.w = current_quaternion[3];
    current_time = msg->header.stamp;

    // set header
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    // set x,y,z
    transformStamped.transform.translation.x = current_x;
    transformStamped.transform.translation.y = current_y;
    transformStamped.transform.translation.z = 0.0;
    // set theta with quaternion
    transformStamped.transform.rotation.x = current_quaternion.x();
    transformStamped.transform.rotation.y = current_quaternion.y();
    transformStamped.transform.rotation.z = current_quaternion.z();
    transformStamped.transform.rotation.w = current_quaternion.w();

    // broadcast transform
    br.sendTransform(transformStamped);
}


void Odometry::integrations(const sensor_msgs::JointStateConstPtr& msg) {
    double vel = sqrt(vx*vx + vy*vy);

    ros::Duration time_difference = msg->header.stamp - current_time;
    double t_s = time_difference.toSec(); //time of sampling

    if (integration_method == 1){ // Euler
        // formulas
        current_x = current_x + vel * t_s * cos(current_theta);
        current_y = current_y + vel * t_s * sin(current_theta);
        current_theta = current_theta + omega * t_s;

        custom_odometry.method.data = "euler";

    } else if (integration_method == 2){ // Runge-Kutta
        // formulas
        current_x = current_x + vel * t_s * cos(current_theta + omega * t_s / 2.0);
        current_y = current_y + vel * t_s * sin(current_theta + omega * t_s / 2.0);
        current_theta = current_theta + omega * t_s;

        custom_odometry.method.data = "runge-kutta";
    }
    /*
    current_quaternion.setRPY(0, 0, current_theta);

    custom_odometry.odom.header.stamp = msg->header.stamp;
    custom_odometry.odom.pose.pose.position.x = current_x;
    custom_odometry.odom.pose.pose.position.y = current_y;

    custom_odometry.odom.pose.pose.orientation.x = current_quaternion[0];
    custom_odometry.odom.pose.pose.orientation.y = current_quaternion[1];
    custom_odometry.odom.pose.pose.orientation.z = current_quaternion[2];
    custom_odometry.odom.pose.pose.orientation.w = current_quaternion[3];
    */


}

//callback to copy the filtered twist to the message in order for the publisher to publish it
void Odometry::callback_publisher_timer(const ros::TimerEvent& ev) {
    if ((latest_sent_time - current_time).toSec() != 0) {
        latest_sent_time = current_time;

        //Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped
        pub_speeds.publish(velocities);

        // ROS parameter for initial pose (x,y,θ) published as custom odometry message
        pub_odom.publish(custom_odometry);
    }
}

bool Odometry::callback_set_odometry(robo::set_odometry::Request &request, robo::set_odometry::Response &response) {
    new_x = request.x;
    new_y = request.y;
    new_theta = request.theta;
    reset = true;
    return true;
}

void Odometry::callback_dynamic_reconfigure(parametersConfig &config, uint32_t level) {
    integration_method = config.integration_method;
}

/*
* omega: angular velocity
* vx: linear velocity along x [m/s]
* vy: linear velocity along y [m/s]
* w1,w2,w3,w4: angular velocities of the wheels [rad/s]
* l,w = robot dimension
* r = wheel radius
*/
void Odometry::computeOmega() {

    w1 = (-(l + w) / r) * omega + vx / r - vy / r;
    w2 = ((l + w) / r) * omega + vx / r + vy / r;
    w3 = ((l + w) / r) * omega + vx / r - vy / r;
    w4 = (-(l + w) / r) * omega + vx / r + vy / r;

}
/*
* La formula per il calcolo di vbx,vby e omega è riadattata alle nostre convenzioni consierando
* che in quel PDF trovato da Simone scambiava la ruota 3 con la ruota 4
*/
void Odometry::computeVelocities() {

    vx = (w1 + w2 + w3 + w4) * (r / 4.0) / 60.0 / gear_ratio;
    vy = (-w1 + w2 - w3 + w4) * (r / 4.0) / 60.0 / gear_ratio;
    omega = (-w1 + w2 + w3 - w4) * (r / 4.0 / (l + w)) / 60.0 / gear_ratio;

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_computation");

    Odometry odometry; // calls constructor

    ros::spin(); // needed

    return 0;

}
