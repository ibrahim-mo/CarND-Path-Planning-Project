## Writeup for the Path Planning Project

#### By: Ibrahim Almohandes, Nov. 11, 2017


The goal of this project is to safely drive a vehicle around a virtual highway with other traffic driving around the speed limit of **50 MPR (+/-10 MPR)**. The vehicle tries to efficiently drive close to the speed limit, passing slower vehicle and/or slowing down as necessary.


### Vehicle Trajectory:

The trajectory the car is following on the virtual highway satisfies all the following requirements:

- **The car is able to drive at least 4.32 miles without incident**: The car completes a full loop (about 7 km) - and even beyond that - without any incidents.

- **The car drives according to the speed limit**: The car drives very close to the speed limit (> 49 mph) most of the time, unless there's a slow traffic which the car tries to pass and/or slow down, as necessary.

- **Max Acceleration and Jerk are not Exceeded**: The acceleration and jerk are below the requirements of `10 m/s^2` and `10 m/s^3` respectively.

- **Car does not have collisions**: No collisions are reported by the simulator.

- **The car stays in its lane, except for the time between changing lanes**: Vehicle stays in its lane unless it needs to change lane, which happens very quickly and smoothly (in less than 3 seconds).

- **The car is able to change lanes**: The car smoothly changes lanes to pass slower vehicles, and does so only when there's enough space to do the lane change maneuver.


### Car model:

I am giving here a brief description of the techniques I followed to drive the car safely and efficiently around the road.

First, the model reads the map coordinates (from a file), then - roughly at every cycle - the model reads from the simulator the car localization and sensor fusion data. The model predicts trajectory points that represent the vehicle's desirable path and passes them back to the simulator at the end of each cycle (of the _`lambda`_ function - lines **250-452**). The simulator processes some of these prediction points, then returns back the remaining points (`previous_path`) which we benefit from to start our new path instead of recalculating all the points from scratch. What we do next is to find the safest and most efficient path that continues from the previous path points. Only at the very beginning we need to calculate all the path points.

To form the rest of the path, we try to find the safest and most efficient path possible. The code that does this is at lines **340-377** of `main.cpp`. In this section of the code, we have two overlapping loops (representing 9 states): one loops over the lanes that the car can be directly driven into (current lane, its left lane - if any - and its right lane - if any), and the other loops through different speed increments (current reference speed, another lower by 0.4 miles, and a third higher by 0.4 miles). The 0.4 miles speed shift is chosen as to satisfy the maximum acceleration and jerk limits of `10 m/s^2` and `10 m/s^3` respectively:

```
Change in speed = (0.4 mi * 1.602 km/mi * 1000 m/km) / (1 hr * 3600 s/hr) = 0.178 m/s
-> Acceleration = (0.178 m/s) / (0.02 s) = 8.9 m/s^2
-> Jerk (in one second) = (8.9 m/s^2) / (1 s) = 8.9 m/s^3
```

This section of the code (i.e., lines **340-377**) depends on two functions to make its decision about which lane and speed it selects to create the new trajectory from (in addition to the previous points passed by the simulator, if any):

- `check_collision()` - defined at lines **164-189** of `main.cpp`: This function checks for two situations: collisions and an arbitrary buffer space in front and back of our vehicle (at each of the three lanes - at most - that we pass to the function one at a time). By excluding collisions and smaller than buffer trajectories from the beginning, we don't need to write a cost function for them. It's actually safer this way!

- `calc_cost()` - defined at lines **194-208** of `main.cpp`: This function combines two cost components as follows:

	- A cost that penalizes speeds that are far from the target speed of `50 mph`. This component has a higher weight (`1000`) for two reasons: the speed ratio can be very small and it has a higher priority than changing or keeping lanes.
	- Another cost that prioritizes lane change behavior to facilitate better decision making (first try to keep lane, then try to change lane to the left, and finally try to change lane to the right). The penalties - before applying weight (`10`) - are `0, 1, and 2` respectively.

Next, based on both the collision/buffer and cost minimizing functions, we settle on the best trajectory which is - for now - represented by the velocity `ref_speed` and the lane number `lane`. Note that in case all the possible paths lead to a collision (flag `collision=true`), we simply stay in lane and reduce the speed (by `0.4` miles). That's probably happens because the buffer was a little high. However, by applying this reduction successively (per each cycle), this will increase the distance back to the buffer zone. This works nicely as no accidents were reported by the simulator even when virtually driving the vehicle for a very long period of time.

The last step - before the `lambda` function hands back control to the simulator - is to create the remaining trajectory points (based on the lane and speed we got from the previous step). Although there are multiple ways to do this (for example, Jerk Minimizing Trajectory (JMT) using quintic function), I found that the open-source library **splice** - defined in `spline.h` - is easy to use and works very nicely. The code that calls this open source library to smoothly create the rest of the trajectory is at lines **383-435** of `main.cpp`. One trick that makes life easier is to convert coordinates from the global or map domain to the local or car domain, then create the slice trajectory, and finally converts them back to the global coordinates. Finally, the trajectory points created by **splice** are added to the `nest_x` and `next_y` arrays, which get passed back to the simulator.

NOTE: The functions that convert from Cartesian to Frenet (and vice versa) are already given, hence I have not discussed them.

Thanks!

Ibrahim
