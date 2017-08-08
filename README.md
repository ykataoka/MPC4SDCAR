# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Vehicle Model and Control Strategy

While there are several well-known vehicle models, the kinematic model is used in this project.
The kinematic model is described with the following equations.
* x\_{t+1} = x\_{t} + v\_{t} cos(\psi\_{t}) dt
* y\_{t+1} = y\_{t} + v\_{t} sin(\psi\_{t}) dt
* \psi\_{t+1} = \psi\_{t} + v\_{t} \delta dt / Lf
* v\_{t+1} = v\_{t} + a\_{t} * dt
where Lf is the vehicle parameter (the distance between the front of the vehile and COG),
delta is the steering angle (control input) and a\_{t} is the acceleration controlled by brake and acclerator.

Since we use the Ipopt for the numeric optimization in this model
prediction control, the nonlinear model can be used as well as dynamic
model. You may tweak the model by considering other factors such as
friction, slip and tire to make the model precise. However, we stick
to the concise model.

## Timestep length and Elapsed Duration (N&dt)
Timestep length is manually chosen, N=20, dt=0.1, based on the following findings through trial and error.

* While (N=20 & dt=0.25) and (N=4 & dt = 0.25) are not successful, N=8 and dt=0.25 works.
  This means that the time duration for MPC considered not to be too long or too short.
  If time duration is too long such as 5.0 second, it can over react to the 5 seconds-ahead course structure.
  If time duration is too short such as 1.0 second, it can not react to the sudden variation such as the steep corner.  
  Thus, I decided to consider the time duration as 2.0 seconds.
* It is obvious that less dt is better as it can consider more points for optimization.
  The downside of it is more computation. Based on the trial and error, dt = 0.1 would be enough for this simulation.
  The system has 0.1 second delay due to the hardware & computation constraint, so the model predictive control should consider at least less than 0.1.


## Reference trajectory
The reference trajectory is fitted to waypoints(given by the simuation based) by a polynomial equation.
For the reference trajectory f(x), the third polynomials are used.
* f(x) = a_3 x^3 + a_2 x^2 + a_1 x + a_0

## Cost function parameter
The parameter is tuned manually too. Initially, all the parameter is set to be 1 though, it did not work in my case due to overreaction to the error of the heading angle. Thus, the cost for the haeding angle parameter is increased and then it became stabilized.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
5. Run Udacity's simulator, MPC Controller