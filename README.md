# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Model Predictive Control
In this project we use a model predictive control (MPC) to predict actuator inputs
for a car. The MPC is given a trajectory at each time step, and by tuning a set of
constraints and cost parameters, we output the actuator inputs that minimizes this
optimization problem.

## The Model
The state has 6 independent variables, the x coordinate, y coordinate, orientation (psi),
velocity, cross track error (cte) and orientation error (epsi),
and their update equations are as follows:

* x_t+1 = x_t + v_t * cos(psi_t) * dt
* y_t+1 = y_t + v_t * sin(psi_t) * dt
* psi_t+1 = psi_t + v_t / L_f * delta_t * dt
* v_t+1 = v_t + a_t * dt
* cte_t+1 = cte_t + v_t * sin(epsi_t) * dt
* epsi_t+1 = epsi_t + v_t / L_f * delta_t * dt

where delta is the steer angle, and L_f is a constant measuring the distance between
the center of mass of the vehicle and its front axle. The larger L_f, the slower the
turn rate.

We constrain the outputs (steer and throttle) with the following ranges:
* delta [-25 * L_f, 25 * L_f] (which translates to a maximum of 25 degree turns)
* a [-1, 1]

For the costs, I applied a larger penalty to cte and epsi errors, by a factor

of 2500, while also penalizing deviation from a reference velocity of 100 mph.
Large actuator values was also penalized, by a factor of 100.

Finally large differences in sequential actuator values were penalized, by a
factor of 200 for delta (steer) and 10 for a (throttle).

## Receding Horizon
A calculation is performed at every time step, and the output trajectory is defined
over a defined number of timesteps (N) of defined duration (dt) which can be tuned.
Ideally, N would be large and dt small, so we can have a more continuous trajectory.
The time horizon, given by N * t, should be tuned to not be too short which results
in myopic trajectories but not too long so as not to be too computationally expensive.

I used an N value of 15 and a dt value of 0.05 which gives a time horizon of 0.75s.
This gave a trajectory that was close to the waypoint trajectory.
Initially I used the recommended value of N=10 and dt=0.1 which gives a time horizon
of 1s, but the large dt resulted in not so accurate trajectories.

## MPC Preprocessing
MPC preprocessing is done by transforming the waypoints to the car coordinate systems

## Polynomial Fitting
A degree 3 polynomial is used to fit the waypoints, and this polynomial is then fed
into the solver to obtain our actuator inputs, as well as the calculated trajectory
of our receding horizon

## Handling Latency
There is latency when we actuate the car with the MPC's output, as time is needed to
perform the calculations. This results in the actuators being applied at a later
timestep then the timestep it was optimized for. To mitigate this latency, we do not
use the actual state calculated at the timestep, but a simulated state after a delay
of 100ms to give as input to the solver.



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

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
