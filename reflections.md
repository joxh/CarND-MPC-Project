# Reflections on Model Predictive Control

## The Model
> Student describes their model in detail. This includes the state, actuators and update equations.

The model that we used was a physical model based on a simple car of fixed length with an actuated steering and throttle.

```
state = x, y, psi, v, cte, epsi
```
* `x`: x position
* `y`: y position
* `psi`: orientation
* `v`: vehicle velocity
* `cte`: cross track error
* `epsi`: orientation error

Futhermore, there are two actuators:
* `delta`: the steering angle, which is limited to +/- 25 deg.
* `a`: The acceleration which is limited to +/- 1 m/s.

The update equations which describe a final state (`state'`) given an initial `state` and an elapsed time `dt` are given by:

```
x' = x + v * cos(psi0) * dt);
y' = y + v * sin(psi0) * dt);
psi' = psi + v * dt * delta / Lf );
v' = v + a * dt;
```

Furthermore, a cost was imposed on the so-called "cross-track error" (cte) and the error in the orientation of the car from the ideal direction

```
cte' = ((f0 - y0) + (v * sin(epsi0) * dt));
epsi' = ((psi - psides0) + v0 * delta0 / Lf * dt);
```

At each time step, the control paramters are determined by solving an optimization problem, where different costs are given to missing certain benchmarks of the calculated 

```cpp
w_cte_err = 2.0; //Cost of CTE
w_epsi_err = 1.0; //Cost of orientation error
w_v_err = 1.0; //Cost of incorrect velocity
```
Then there were costs associated with absolute values of actuator levels. The more actuation, in general the less good the ride.
```cpp
C_delta = 250.0; // Cost of steering actuation
C_a = 5.0; // Cost of acceleration
```
Finally there is a cost for rapid changes in acuation values:
```cpp
C_ddelta = 8000.0; // Cost of steering change
C_da = 10.0; // Cost of acceleration change
```

The system is then solved for an optimal actuation values over a time series of length `N` and time step `dt`;

## Timestep Length and Elapsed Duration (N & dt)
> Student discusses the reasoning behind the chosen `N` (timestep length) and `dt` (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The choice of total number of timesteps and elapsed time between timesteps was made by balancing a few considerations. First of all, I didn't want too many time steps. If N were too large or dt too small, the calculation of optimal `delta` and `a` would have taken too long, and the controller would have failed.

However, there needed to be enough timesteps that the horizon of what the car was considering would be long enough. If it was too short, then the sytem would not be able to see upcoming obsticals. On the other hand, if it was too long, then the car might make bad short term decisions to avoid obsticals that could easily be avoided later with relatively minor adjustments. In general I wanted the car to see about 2 seconds ahead of wherever it was going with as big a density as my system could allow. I therefore settled on `N=7` and `dt=.3`.

## Polynomial Fitting and MPC Preprocessing
> If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

A third order polynomial is fitted to the waypoints so that the state of the car can be easily compared. The waypoints were pre-processed by transforming them from absolute space to the local reference frame of the car. This allowed the fitted curve to be simple and flat close to the car. Also, to do the debugging output of the yellow and green lines, the coordinate system must be that of the car. Furthermore, units for `v` was changed to  meters per second from miles per hour.

The vehicle state is also broght to a point where its initial position is `(x,y)=(0,0)` and its inital orientation is `psi=0` within the instantaneous vehicle coordinate system. 

![A turn](media/self_driving_car_nanodegree_program 9_18_2017 6_01_15 AM.png "Model encountering a curve")


## Model Predictive Control with Latency

> The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Furthermore the latency in control was taken into account in the model. This was done by taking the values of `x`, `y`, `v`, `psi`, `delta` and `a` returned from the simulator. With these values, I could use the same equations to evolve the system forward in time by the time of the latency (0.1 seconds). Since this represents the state of the system when the actuator commands will reach the car, this state was used as the initial conditions for the optimizer. This resulted in a much more stable output for the waypoints, and also a more accurate fit.

```cpp
double px_delay = px + ds * cos(psi);
double py_delay = py + ds * sin(psi);
// Calculate to second order
double psi_delay = psi + (ds * tan(-delta_0) / Lf);
```

Note: This is done immediately after getting the message from the simulator and before the transformation to vehicle coordinates. 