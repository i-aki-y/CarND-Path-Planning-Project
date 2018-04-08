# Path Planning Project

This is a writeup for Path Planning Project.

## About the Project

The goal of this project is to make a car go around the given course with satisfying the following rules.

- The car run at speed less than 50 mph
- The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
- The car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes.

In this project, the path planning model should calculate the next car path points with a certain interval. The interval determines the car velocity because of the car moves to next path point in time $\Delta t = 0.02$ second. The simulator provides the model with other car sensor fusion data and the previous paths information which the model created. The waypoints of the whole course are given as a additional information. The model can use these waypoints in order to make the car drive along the road. The road has six lanes and three of them are counter lanes. Every lane has 4 m width. Figure 1 is an overview of the project configuration.

![Configuration](./img/ProjectConfig.png?)
Figure 1. A configuration of the project.

## About the Code
I defined some additional class. A brief explanation of each class is shown in the followings.

`PathPlanner` class is introduced to manage to create the next path. This class knows how to create the next path from the given previous path and other cars sensor fusion.

`CarInfo` class is a simple data structure, which is used for describing common car information in this project such as Cartesian, Frenet coordinates and velocity of the car.

`SensorFusion` class describes sensor fusion data of single car.

`RoadStatus` class is a class which contains road status around my car.

## Model Documentation

This model based on the lecture walkthrough approach where paths are created by a spline curve. The strategy is described in the followings.

### 1. Path Generation

A spline curve is a smooth curve which goes through the given points called control points. In order to draw a good spline curve, I should select appropriate control points which are selected from previous path points and waypoints. In this model, five control points are selected in the following ways.

The implementation of control point selection is given in the `PathPlanner::CreateControlPoints()` method of the `PathPlanner.cpp` file.

First, The last two points of the previous path, which are given by the simulator, are used as first two control points. If there are no previous path points, current car position and the previous car position guessed from yaw angle are used.

Second, three points are selected from the waypoints of the course. Since the waypoints are sampled at intervals of 30m in 'Frenet Coordinates', I selected the waypoints that exist up to 90 m ahead of the car.
The Frenet Coordinate of these points are explicitly described as
$$
  (s_i, d_i) = (s_c + 30i, (l + 0.5)L_w ),
$$
where $s_i$ and $d_i$ are longitudinal and lateral coordinate of $i$-th selected waypoints. The constant $L_w$ means lane width (m) and the variable $l$ is the number of lane where $l=0, 1, 2$ are indicating the left, middle and right lanes, respectively.
Note that the $l$ is the **target** lane number and it does not always indicate the lane in which the car is running.

With letting $l_c$ the current lane number (the number of the lane in which the car is running), control points which indicate a path switching lane is obtained by setting $l = l_c \pm 1$.

In order to define spline curves as a function of $x$ coordinate, a coordinate transformation is applied. In the new coordinate, the yaw angle and potion of the last previous path point are set in the direction of $x$-axis and the origin, respectively. I use $x', y'$ variable as a new $x, y$-coordinate. Therefore, the spline curve is described by $y' = f(x')$, where $f$ is a spline function determined by control points.

The coordinate transformation is defined in `void TransformCoordinate()` method of `Utils.h` file and applied in `PathPlanner::CreateNextPath()` method of `PathPlanner.cpp` file.

After defining the spline curve from the selected five points, the next path points which will be added to the previous path are picked up from the spline curve at a certain interval. The interval $\Delta r$ is given by
$$
\Delta r = v_{\rm{ref}}\, \Delta t,
$$
where $v_{\rm{ref}}$ is the target velocity because the car velocity is determined by the interval of the path points.

In order to determine the $x', y'$ of the interval $\Delta r$, I select a curve in a small region $[0, x_a']$ in $x'$-axis and approximate the distance along the curve with the distance $r$ between $(0, 0)$ and $(x_a', f(x_a'))$, which is given by
$$
  r = \sqrt{(x_a')^2 + f(x_a')^2}.
$$
The number $N$ that the distance $r$ could contain the interval $\Delta r$ is
$$
 N = r / v_{\rm{ref}}\, \Delta t.
$$
Therefore, the interval of $x', y'$ is obtained by
$$
\begin{aligned}
  \Delta x' &= x_a'/N,\\
  \Delta y' &= f(\Delta x').
\end{aligned}
$$

![SplineSample](./img/SplineSample.png?)
Figure 2: Spline curve is represented by $y'=f(x')$.  The length of the curve between $(0, 0)$ and $(x_a', f(x_a'))$ is approximated by the distance $r$.

Additional $i$-th path points are given by
$$
  (x_i', y_i') = (i\Delta x', f(i\Delta x'))
$$

The implementation of next path points are given in the `PathPlanner::CreateSplinePath()` method of `PathPlanner.cpp` file. I used `spline.h` to create spline curve.[^sp]

### 2. Lane Switching.

If my car finds any other car running ahead in a given threshold distance, my car will decide (1) switch lane and pass the other car, or (2) slows down and follows the ahead car.

Whenever there is no other car in another lane around my car, my car will switch the lane. The criteria whether another car exist or not is introduced by $[s_c - b, s_c + a]$ in $s$-coordinate of Frenet Coordinate, where $s_c$ is the car $s$-coordinate, and $a$ and $b$ are parameter of the model. If large values are set $a$ and $b$, my car will be more defensive, since my car will decide to switch lanes only the time when there is no car over a wide region.

Figure 3 shows that situations. In the case of Figure (a) where my car (the black one) can find in the right lane that there is no car in the region of $[s_c-b, s_c + a]$ while in the left region another car (yellow one) exists, so that my car can switch the lane. On the other hand, in the case of Figure (b), both lanes are filled by other cars. Therefore my car stays middle lane and follows the other car in front.

![SwitchLane](./img/SwitchLane.png)
Figure 3: (A) My car found other car driving ahead and that there is no car in the right lane while the left lane has a yellow car in the region. In this case, my car will switch to the right lane. (B) My car found other car driving ahead, but the left and right lanes are filled with other cars. Therefore my car stays middle lane and follows the car in front.

It seems to be more advantageous that the car is driving in the middle lane rather than driving right or left lane, since the car driving middle lane has two options switching left or right. Hence, I make my car prefer to switch to middle lane if it running in right or left lane and it can switch (See, Figure 4).

![SwitchLane](./img/SwitchToMiddleLane.png)
Figure 4: If the car runs in the left or right lanes and there is no car in the middle lane. The car will switch to the middle lane.

To simplify the model's rule, I imposed the rule that the car could not change the target lane after the car start to switching until the lane switching has completed.


Estimation of road status is implemented in the  `PathPlanner::UpdateOtherCarStatus()` method of the `PathPlanner.cpp` file. The `PathPlanner::IsFilled()` method determines whether any other car exist in the region of evaluation criteria or not. How target lane number is changed according to the evaluated road status is given in the `PathPlanner::UpdateTargetLaneNum() ` method of `PathPlanner.cpp` file.


[^sp]: http://kluge.in-chemnitz.de/opensource/spline/
