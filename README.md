<p align="center">
  <img width="500" src="images/polimi_logo.png" alt="PoliMi Logo" />
  <br>
  <br>
</p>

# Robotics Project 1 - Academic Year 2021/2022 - Prof. Matteo Matteucci

## Group members //links to our githubs
- [__Marco D'Antini__](https://github.com/DantiniMarco) (10603556)
- [__Simone Giampà__](https://github.com/SimonGiampy) (10659184)
- [__Gabriele Pagano__](https://github.com/gabrielepagano) (10578117)


## Files inside the archive 
|File|Description|
|---------------|-----------|
|__parameters.cfg__|dynamic parameters and methods to handle their reconfiguration|
|__files of 'images' folder__|snapshots and images useful for the README|
|__Control.h__|header file for the Control class|
|__Odometry.h__|header file for the Odometry class|
|__odometry.launch__|launch file that runs the whole project (except the bags)|
|__odom.msg__|message file containing the odometry to be published|
|__record.msg__|message file containing the records of the bags (it is used for calibration)| 
|__wheels-rpm.msg__|message file containing the wheel velocities to be published|
|__estimator.py__|Python code written to calibrate parameters|
|__Control.cpp__|publishes the wheel velocities given the robot angular and linear velocities|
|__Odometry.cpp__|sets the service for odometry setting and the dynamic reconfigure. Moreover it publishes the robot velocities given the wheel speeds, the bags recorder and the robot odometry (see following points for further details)|
|__set-odometry.srv__|file containing the parameters to be set thourgh the \_/set_odometry\_ service|



## ROS parameters 

### Service reconfigurable parameters
- __x:__ used to set the x coordinate of the odometry
- __y:__ used to set the y coordinate of the odometry
- __theta:__ used to set the theta coordinate of the odometry

### Dynamically reconfigurable parameters
- __EULER:__ used to set the current integration method to Euler
- __RUNGE-KUTTA:__ used to set the current integration method to Runge-Kutta


## Structure of the TF tree //is this the correct one?
- After have run the file bag1.bag, we visualized the structure of the TF tree thanks to the following command:

```
rosrun rqt_tf_tree rqt_tf_tree
```
- Thus, we obtained the following:

<p align="center">
  <img src="images/tf_tree.png" alt="TF Tree" />
  <br>
  <br>
</p>


## Structure of custom messages 

### odom.msg
- __nav_msgs/Odometry odom:__ odometry published in terms of orientation (with the quaternion <x,y,z,w>)
	and position (with the tuple <x,y,z>)
- __std_msgs/String method:__ method of integration currently used (Euler or Runge-Kutta)

### record.msg
- __header:__ contains informations about timestamp and sequence ID of the publication
- __pose_x:__ x coordinate recorded on the \_/robot/pose\_ topic
- __pose_y:__ y coordinate recorded on the \_/robot/pose\_ topic
- __pose_theta:__ theta coordinate computed after have been recorded the orientation in the \_/robot/pose\_ topic
- __w1:__ velocity of the front left wheel recorded in the \_/wheel_states\_ topic
- __w2:__ velocity of the front right wheel recorded in the \_/wheel_states\_ topic
- __w3:__ velocity of the rear left wheel recorded in the \_/wheel_states\_ topic
- __w4:__ velocity of the rear left wheel recorded in the \_/wheel_states\_ topic

### wheels_rpm.msg
- __header:__ contains informations about timestamp and sequence ID of the publication
- __rpm_fl:__ velocity of the front left wheel (computed from robot velocities) expressed in revolutions per minute
- __rpm_fr:__ velocity of the right left wheel (computed from robot velocities) expressed in revolutions per minute
- __rpm_rr:__ velocity of the rear right wheel (computed from robot velocities) expressed in revolutions per minute
- __rpm_rl:__ velocity of the rear left wheel (computed from robot velocities) expressed in revolutions per minute

### set_odometry.srv //Have we to put also this between custom messages?
- __x:__ x coordinate to be set by the service \_/set_odometry\_
- __y:__ y coordinate to be set by the service \_/set_odometry\_
- __theta:__ theta coordinate to be set by the service \_/set_odometry\_

## How to start nodes
- To start all the nodes is necessary to open a terminal and type the following command:

```
roslaunch odometry_project odometry.launch
```

## How to configure parameters //dunno if keep dynamic config info or not
- After the nodes have been launched it is possible to configure ROS parameters.
- To configure the parameters <x,y,theta> through a service open a terminal and type the following command:

```
rosservice call /set_odometry (new_x) (new_y) (new_theta)
```
__NB:__ one has to replace (new_x),(new_y) and (new_theta) with the values he wants to set for the odometry

- One way to configure dynamic parameters is to open a terminal and type:

```
rosrun rqt_reconfigure rqt_reconfigure
```
__NB:__ after that a new window will be opened and the user can dynamically configure the integration method 

## Further informations
- Publications on the topics are performed periodically thanks to a timer
- We decided to use revolutions per minute (instead of radians per minute) as unit measure for wheel velocities computed from robot velocities (vx, vy and rotational speed omega)
- As we noticed some recurrent dependencies in the formulas between the robot parameters (i.e., 1/(l + w) and r/(4 x gear_ratio)), we decided to calibrate at first these two new parameters by multiple iterations on certain ranges and then calibrate r and w keeping T and l fixed 
