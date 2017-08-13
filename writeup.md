# Writeup for Model Predictive Control Project
My 10th Project in the Self-Driving Car Engineer Nanodegree at Udacity

---

### Student describes the model in detail. This includes the state, actuators and update equations.

The model uses six state variables. Each variable contains a value from the vehicle's coordinate system. The state variables describe the current state of the vehicle and the actuator variables contain values to influence the state variables in the right way.

State variables:<br>
**x**: The x position<br>
**y**: The y position<br>
**psi**: The orientation of the vehicle in radians<br>
**v**: The vehicle's velocity in mph<br>
**cte**: The **C**ross **T**rack **E**rror which is the difference between the vehicle's position and the optimal position<br>
**epsi**: The error of the vehicle's orientation compared to the optimal orientation

Actuator variables:<br>
**delta**: The vehicle's steering angle<br>
**a**: The vehicle's acceleration

The update equations are:<br>
x1 = x0 + v0 * cos(psi0) * dt<br>
y1 = y0 + v0 * sin(psi0) * dt<br>
psi1 = psi0 + v0 * delta0 / Lf * dt<br>
v1 = v0 + a0 * dt<br>
cte1 = f0 - y0 + v0 * sin(epsi0) * dt<br>
epsi1 = psi0 - psides0 + v0 * delta0 / Lf * dt<br>

**dt** is the time difference between the last and the current state.<br>
**Lf** is the distance between the front of the vehicle and its center of gravity.<br>
**f** is the optimal waypoint function.<br>
**psides** is the derivative of psi.

### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

For the final model, I chose the following values:<br>
**N**: 8<br>
**dt**: 0.125

I always changed the two values relative to each other. At first, I chose 0.1 for **dt** due to the simulator's latency of 100 milliseconds. **N** was set to 10. Lower values for **dt** led to a car which turned the steering wheel too much and the model worked fine with **dt=0.1** and **N=10**. Especially the trajectory line looked really smooth. But after I tuned the model, I raised **dt** to 1.25 and lowered **N** to 8. It made the car drive smoother in general, even though it went closer to the ledges in curves. The trajectory line does not look that smooth anymore, but the car's smoother driving behavior looks more comfortable for passengers.

### If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints and the vehicle state where privided as values of the global coordinate system. I transformed these values to the vehicle's coordinate system before handing it to the MPC.

The transformation of the waypoints is calculated by the following formulas with local_vehicle_psi, x and y being 0:

transformed_x = (waypoint_x - vehicle_x) * cos(local_vehicle_psi - global_vehicle_psi)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- (waypoint_y - vehicle_y) * sin(local_vehicle_psi - global_vehicle_psi)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;+ x

transformed_y = (waypoint_x - vehicle_x) * sin(local_vehicle_psi - global_vehicle_psi)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;+ (waypoint_y - vehicle_y) * cos(local_vehicle_psi - global_vehicle_psi)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;+ y

After the transformation, a polynomial is fitted through the waypoints. The **velocity** does not need transformation. The **cte** is the value of the polynomial at x=0 minus y (which is also 0). The **epsi** value equals y - atan(coeffs[1]) with y being 0.

Since local_vehicle_psi, x and y are equal to 0, the state vector looks as follows:

( 0.0, 0.0, 0.0, v, coeffs[0], -atan(coeffs[1]) )

### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

I dealt with the latency by estimating the vehicle's current position.

x = x + v * cos(psi) * dt<br>
y = y + v * sin(psi) * dt

I used 0.05 for **dt** instead of 0.1. I figured that the estimation always overshoots when using 0.1. This is probably due to the computation time the computer needs for the MPC calculations. The value **dt=0.05** estimates the current position pretty good. Estimating the vehicle's current orientation made the vehicle driving worse, so I decided to keep the psi value from the measurement.

### Result

The car stays permanently on the road and drives with about 60 mph around the track. The result can be watched in this [video](https://drive.google.com/open?id=0B0agIiDyIPj1VVVLcTg5Sk9LS28).
