# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
The project illustrates the implementation of Model Predictive Control (MPC) controller that enables a car to follow a trajectory provided by path planning module.

[![IMAGE ALT TEXT HERE](Demo.gif)](https://youtu.be/F_VOoaFvCP4)

This MPC was tunned to achieve a more aggressive driving style. The car in this demo reaches speeds of 90mph and slows down quickly to as low as 30mph. The performance of MPC is comparable to a competitve human driver. All of the decision regarding steering angle and the optimal speed are made by MPC based on a simple kinematic model of a car. The model can be further improved by incorporating more data related to proximity to sounding obstacles as well as by extending the model to incorporate a dynamic model of the vehicle.

---


## Rubric / Project Specification

### The Model
The model used in the project is a global kinematic model of a vehicle.
**States** - The model assumes that a vehicle state represented by 4 coordinates [x,y, rotation(psi). velocity(v)]
**Actuators** - The state of a vehicle depends only on control inputs of actuators [steering(d), acceleration(a)]
**Update equations** - The control inputs change the state of the car as follows:

x(t+1) = x(t) + cos(psi)*dt

y(t+1) = y(t) + sin(psi)*dt

psi(t+1) = psi(t) + v(t)/Lf * d * dt

v(t+1) = v(t) + a(t)*dt

To find appropriate controller (actuator) values, MPC minimizes a cost function wrt constraints. The const function defined as:

∑G(d,a,x) = w_cte * cte(t+1) + w_epsi * epsi(t+1) + w_psi * (psi(t+1)-psi(t)) + w_d * d(t) + w_tv * (100mph - v(t+1))

Where:

cte - defined as a cross track error of position y(t+1) and trajectory value f(x(t+1))

epsi - defined as orientation error and being calculated as: epsi = atan( f'(x(t+1)) ) - psi(t+1). Where f'( x(t+1) ) is a derivative of trajectory curve


The corresponding code can be found in **MPC.cpp - lines 78 to 113**.

### Timestep Length and Elapsed Duration (N & dt)

MPC is highly sensitive to the settings of various components of cost function as well as dt and N.
**dt** - the duration of the time stamp was set at 0.1 seconds to run efficiently with the goal to cover a trajectory of roughly 1 second.
N - the number of steps is the main driver of the computational cost associated with the algorithm. Therefore, the N was set based on the length of projected trajectory. The best behavior was achieved using 10 steps.

### Data preprocessing steps

x,y, and psi observations were preprocessed with the inverse of homogenious transform to convert observations from a map coordinate system into a car coordinate system. Velocity value was converted from mph to meters per second.
The overall trajectory was calculated by fitting 3rd-degree polynomial function onto waypoints (transformed into car coordinate system) provided by a simulator/path planning block.


The corresponding code can be found in main.cpp, lines 107-117.


### Model Predictive Control that handles a 100-millisecond latency

In order to adjust for latency in engaging the actuators, the model calculates the new state at the time t+100 milliseconds. The values of the new state are used subsequently to transform coordinates of waypoints. This setup allows controlling the car successfully at various latencies in a range between 0 and 100 milliseconds.


The corresponding code can be found in main.cpp, lines 98-107.

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
