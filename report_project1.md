# Robotics Project 1 - Academic Year 2021/2022 - Prof. Matteo Matteucci

## Contributors
- [__Marco D'Antini__](https://github.com/DantiniMarco) (10603556)
- [__Simone Giampà__](https://github.com/SimonGiampy) (10659184)
- [__Gabriele Pagano__](https://github.com/gabrielepagano) (10578117)

## Content
Our project is made up of __two  classes__: 
- ``Odometry.cpp`` where we compute the odometry part;
- ``Control.cpp`` where we compute the control part;
- ``Odometry.h`` and "Control.h" are header file where all the declarations are made.
- ``estimator.py`` is a python script that we used to deal with the robot dimensions calibration part.

#### ``Odometry.cpp``

We wrote down the __formulas__ to compute velocities along x, y (_vx_, _vy_) and angular velocity (_omega_) given the wheel velocites (_w1_, _w2_, _w3_, _w4_) taken from the bags. The following: 
```
    vx = (w1 + w2 + w3 + w4) * (r / 4.0) / 60.0 / gear_ratio;
    vy = (-w1 + w2 + w3 - w4) * (r / 4.0) / 60.0 / gear_ratio;
    omega = (-w1 + w2 - w3 + w4) * (r / 4.0 / (l + w)) / 60.0 / gear_ratio;
```
these formulas are corrected for our purposes and __unit of measurement__:
- division by 60.0 to transform __[rad/min]__ to __[rad/s]__;
- division by _gear_ratio_ to adapt the transmission ratio.

In particular the wheel velocities are imported from the bags using a ROS subscriber named ``sub`` (topic ``/wheel_states``), then ``wheel_state_callback`` is called at each new value, so it computes _vx_, _vy_, _omega_ using ``computeVelocities`` function starting from _w1_, _w2_, _w3_, _w4_;
 
Calling ``integrations`` we check the __method of integration__ required (between __Euler__ and __Runge-Kutta__) reading a field in the message set by the __parameter reconfigurion__ and finally compute the odometry, filling up an ``odom.msg`` named ``custom_odometry`` containing:
``` 
pose.pose.position
pose.pose.orientration
```
where the orientations are set up using a __quaternion__.
These messages are published by a ROS publisher named "pub_odom".
We decided to use a function ``callback_publisher_timer`` to publish our messages each time a __timer__ expires.

The function ``callback_dynamic_reconfigure`` is used to choose the integration method in run-time, so it can switch between Euler and Runke-Kutta approximations.

The callback ``callback_set_odometry`` instead changes the robot pose reading the Request/Response fields of a Service type message named ``set_odometry.srv``.

The methods mentioned before are the main functions we used to accomplish the odometry part of the project. Other methods are for __utility and test purposes__.

In particular we used ``optitrack_callback`` to __verify and confront__ the shape of the velocitiy plots we calculated with a plot that we could have obtained by dividing the delta positions (_delta_x_, _delta_y_, _delta_theta_) by the time of sampling. 
This plot was firstly __too noisy__, so we used an average of every 100 samples using a vector data structure. 
Building and ``test_msg`` message, we plotted it in PlotJuggler and we confirmed that it was correct. 

A similar modality was made in ``encoder_ticks_callback`` where we show the velocity plot using the __ticks instead of RPM__. With the same method of 100 samples average measurement we filled up and published a ``tick_msg`` message in order to visualize it in PlotJuggler. 
Observing a much higher noise shape of the plot, we decided too __keep using the RPM data__ for our odometry.

At this point of the project we started thinking on the __parameters calibration part__, so we decided to develop a Python script in which we compute again the odometry, but this time with the possibility to change and iterate on the required parameters with the aim of __calculating the error value__ with the least squares function. 
In order to do this we needed to record a __new bag__ containing the wheels velocities (_RPM_) and the robot pose, but this time synchronously. In fact we noticed that the _timestamp_ was different between the two measurements. That's what ``record_callback`` is for. It publishes a new message ``record.msg`` through ``pub_record`` ROS publisher. 
We finally recorded the new bag with the values synchronized between the two topics. 

#### ``Control.cpp``

The basic idea is to develop a __reverse computation__ starting from the robot velocities _vx_, _vy_, _omega_ to obtain the wheel velocities. In order to do that we wrote down the following formulas: 
``` 
    w1 = ((-(l + w) / r) * omega + vx / r - vy / r) * k;
    w2 = (((l + w) / r) * omega + vx / r + vy / r) * k;
    w3 = ((-(l + w) / r) * omega + vx / r + vy / r) * k;
    w4 = (((l + w) / r) * omega + vx / r - vy / r) * k;
```    
We multiply per ``k = 60 * gear_ratio / 2 * pi`` to obtain the correct unit of measurement that is [_rad/s_].

These are the __steps__: 
 
1) __subscription__ with a ROS subscriber ``sub`` to the ``/cmd_vel`` topic. This is the topic that we publish in the previous step;

2) at each new incoming message the ``wheel_velocities_callback`` function is called. Here we __take the values__ _vx_, _vy_, _omega_ from from the message;

3) ``computeOmega`` function is called to __compute the angular velocities__ with the actual values;

4) a custom message ``omega_msg`` __is published__ containing the values _w1_, _w2_, _w3_, _w4_ using the ROS publisher ``pub`` at every time the timer expires;

Observing on PlotJuggler the plot of our velocities and the velocities of the bags, we made sure that the __two plots were identical__. 

 




 


 




 


