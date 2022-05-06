# Robotics Project 1 - Academic Year 2021/2022

<p align="center">
  <img width="500" src="images/polimi_logo.png" alt="PoliMi Logo" />
  <br>
</p>

## Contributors

- [__Marco D'Antini__](https://github.com/DantiniMarco) (10603556)
- [__Simone Giampà__](https://github.com/SimonGiampy) (10659184)
- [__Gabriele Pagano__](https://github.com/gabrielepagano) (10578117)

## Project content
Our project is made up of __two  classes__:
- `Odometry.cpp` where we compute the odometry part.
- `Control.cpp` where we compute the control part.
- `Odometry.h` and `Control.h` are header files where all the variables and functions declarations are present.
- `estimator.py` is a Python script that we used to deal with the robot dimensions calibration part.

More information about the contents of this project repository is present in the README.md file of the project

---

## ``Odometry.cpp``

We wrote down the __formulas__ to compute velocities along x, y (_vx_, _vy_) and angular velocity (_omega_) given the wheel velocites (_w1_, _w2_, _w3_, _w4_) taken from the bags expressed in ticks . These are the formulas used:

```
    vx = (w1 + w2 + w3 + w4) * (r / 4.0) / gear_ratio;
    vy = (-w1 + w2 + w3 - w4) * (r / 4.0) / gear_ratio;
    omega = (-w1 + w2 - w3 + w4) * (r / 4.0 / (l + w)) / gear_ratio;
```

these formulas are corrected for our purposes and __unit of measurement__:
- division by _gear_ratio_ to adapt the transmission ratio.
- ticks data are manipulated to obtain a velocity expressed in __[rad/s]__ with the following formula:

```
    v[i] = (ticks_t[i] - ticks_prev[i]) * 2.0 * M_PI / ticks_count / delta_t;
```

where v[i] is the speed of the i-th wheel in rad/s; `ticks_t[i]` is the ticks count at time t, and `ticks_prev[i]` is the ticks count at time t-1; `ticks_count` = number of ticks on the encoder for each wheel;

In particular the wheels' tick positions are imported from the bags using a ROS subscriber named ``sub`` (topic ``/wheel_states``), then ``wheel_state_callback`` is called at each new value, so it computes _vx_, _vy_, _omega_ using ``computeVelocities`` function starting from the ticks and the relative obtained wheel velocities _w1_, _w2_, _w3_, _w4_.

By calling ``integrations`` we check the __method of integration__ required (between __Euler__ and __Runge-Kutta__) reading a field in the message set by the __parameter reconfiguration__ and finally compute the odometry, filling up an ``odom.msg`` named ``custom_odometry`` containing:

```
pose.pose.position
pose.pose.orientation
```

where the orientations are defined using a __quaternion__, and the position is in euler coordinates.
These messages are published by a ROS publisher named `pub_odom`.

- We decided to use a function ``callback_publisher_timer`` to publish our messages each time a __timer__ expires.

- The function ``callback_dynamic_reconfigure`` is used to choose the integration method at run-time, so it can switch between Euler and Runke-Kutta approximations.

- The callback ``callback_set_odometry`` instead changes the robot pose reading the Request/Response fields of a Service type message named ``set_odometry.srv``.

- The methods mentioned before are the main functions we used to accomplish the odometry part of the project. Other methods are for __utility and test purposes__.

- In particular we used `optitrack_callback` to __verify and confront__ the shape of the velocity plots we calculated, with a plot that we have obtained by dividing the robot pose delta values (_delta_x_, _delta_y_, _delta_theta_) by the time of sampling (which corresponds to the derivative of the robot pose by time).
This plot was at first __too noisy__, so we used an average of every 100 samples using a vector data structure.
Building and ``test_msg`` message, we plotted it in PlotJuggler and we confirmed that the 2 plots were matching, so the results are correct.

- We used the function ``encoder_ticks_callback`` to show the two velocity plots in order to __confront ticks and RPM__. After a first attempt that showed us the ticks measurement too noisy we tried to smooth them with a method of 100 samples average measurement we filled up and published a ``tick_msg`` message in order to visualize it in PlotJuggler.
A not significant improvement with this modality led us to keep using the measure of ticks taken from the bags.

- At this point of the project we started thinking on the __parameters calibration part__, so we decided to develop a Python script in which we compute again the odometry, but this time with the possibility to change and iterate on the required parameters with the aim of __calculating the error value__ with the least squares function.
In order to do this we needed to record a __new bag__ containing the wheels tick counts (_ticks_) and the true robot pose, while making sure that they are synchronized. In fact we noticed that the _timestamp_ may be different between the two measurements. That's what ``record_callback`` is for. It publishes a new message ``record.msg`` through ``pub_record`` ROS publisher. Every message published contains values that are coherent with the relative time instant.
We finally recorded the new bag with the synchronized values between the two topics.

---

## ``Control.cpp``

The basic idea is to develop a __reverse computation__ starting from the robot velocities _vx_, _vy_, _omega_ to obtain the wheel velocities. In order to do that we wrote down the following formulas:

```cpp
    w1 = ((-(l + w) / r) * omega + vx / r - vy / r) * k;
    w2 = (((l + w) / r) * omega + vx / r + vy / r) * k;
    w3 = ((-(l + w) / r) * omega + vx / r + vy / r) * k;
    w4 = (((l + w) / r) * omega + vx / r - vy / r) * k;
```    

We multiply by ``k = 60 * gear_ratio / 2 * pi`` to obtain the correct unit of measurement that is [_revolutions/minute_].

These are the __steps__:

1. __Subscription__ with a ROS subscriber sub to the `/cmd_vel` topic. This is the topic that we publish in the previous step;

2. At each new incoming message the `wheel_velocities_callback` function is called. Here we take the values _vx_, _vy_, _omega_ from from the message;

3. `computeOmega` function is called to __compute the angular velocities__ with the actual values;

4. a custom message `omega_msg` __is published__ via a timer callback containing the values _w1_, _w2_, _w3_, _w4_ using the ROS publisher `pub` at every time a timer expires;

Observing on PlotJuggler the plot of our velocities and the velocities taken from the input bags, we made sure that the __two plots were identical__.

---

## Python script for the estimation of the optimal robot dimensions values

#### Input dataset

A bag containing all data has previously been recorded, and is used as a __dataset__ in order to process all data at once instead of waiting for incoming messages via a classical ROS subscriber. The recorded bag contains robot pose values and the wheels speeds values. A dataset is then created and transformed in a __*Pandas Dataframe*__, so data is easily elaborated into arrays, with these lines:

```python
# reads dataset from recorded bag (data from bag3)
b = bagreader('../../../bags/dataset_rec.bag')
# read topic with all data
odo = b.message_by_topic('/recorder')
# memorize data read from bag in a pandas Dataframe
csv = pd.read_csv(odo)
```

#### Values to be optimized

This script aims to find the __optimal values__ for the *r*, *l*, *w* and *N* values. The *gear ratio* is assumed to be fixed at 5. The __dimensions of the robot__ are optimized with 3 variables: *lw* as the sum *l + w* since they appear always as sum in the used formulas, then *r* and *N*. This script iterates over all possible combinations of these values to find the ones which minimize the difference with respect to the true pose.

#### Algorithm used for value optimization

The __algorithm optimizes the values with grid search__, that is, tries all combinations of values for r, lw, N and computes the error compared to the expected values represented by the true robot pose.
In the end it checks the goodness of the approximation visually with the aid of some plots. It follows these steps:

1. starts from 3 lists of values to be tested, one list for each variable to be optimized. The values in the lists vary in a certain range, which is close to the values given in the project assignment (r = 0.07 m, lw = 0.369 m, N = 42).
2. creates an instance of the `Estimator class`, sets the input values, and computes the odometry with the Runge-Kutta approximation method since it is more accurate.
3. computes the __residual sum of squares error__ for the computed odometry and returns the obtained error value.
4. repeat steps 2-3-4 for each combination of the 3 values in the lists and calculate the error with respect to the true pose.
5. Calculates the minimum error value among all simulations. The __optimal value__ for the dimension parameter corresponds to the minimum error.
6. shows 3 plots: one plot for every axis to be compared: x, y, theta. With this way there is a __quick visual check__ of the error function and its minimum value.

#### Error function

The function used for the computation of the error value evaluates the __"goodness"__ of the assigned values to the parameters. It compares the given robot pose (used as a comparison, since it is assumed to be the true pose) with the pose computed with the odometry formulas.
- The __comparison__ is based on the __"least squares" optimization method__: the error is equal to the squared of the difference between the true robot pose and the calculated one. This corresponds to the residual sum of squares metric for this multi-variable optimization problem.
- The smaller the error, the better the approximation.
- The __comparison__ evaluates both x and y coordinates, and assigns equal weight to both of them. The orientation is not considered here due to changing signs in the given orientation values.

#### Final results and discussion

- The input values considered for the plots are the ticks positions, as requested by the project assignment. We tried also to use the RPM velocities values, and we concluded that __this data was much less noisy__ that the ticks data. This means that using 2 consecutive messages for the computation of the wheel speed from the ticks count retrieved by the wheels encoders leads to very noisy data. Instead the RPM values were much smoother. An alternative could have been to create a moving average of the ticks positions so to artificially smooth the input data, but it would result in a different approach to the problem that is not requested for this project.

- These plots show a visual check of the error function and the obtained minimizing value for both variables. These minimizing values are then combined together to show 3 new plots with the comparison between the robot pose and the computed one, with respect to the x, y, theta coordinates.

- Considering the approximation error due to the integration method used, and the not really precise data present in the given bags, our minimizing values show an __almost perfect approximation__ of the true comparison plots.

> Simulation results:
>
> l = 0.2 m (fixed for simplicity)
>
> w = 0.1579 m
>
> r = 0.07579 m
>
> gear_ratio = 5 (fixed)
>
> N = 42

- The calibration of the robot dimension values was accomplished by using as input values both the input velocities expressed in RPM and the input tick positions of the wheels encoders. We noticed that the optimal values were differing and showing different resulting plots, especially for the bag2. Specifically, using the optimal values for the bag2 calculated from the more precise RPM values, we obtained a plot of a completely wrong trajectory. This does not make any sense from a theoretical standpoint, so we assume that the ticks position in the given bag were incorrect, or either the ticks were so noisy that lead to wrong numerical results, despite the correctness of the applied model.
