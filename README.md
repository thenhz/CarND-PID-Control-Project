# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID reflections and writeup

The PID controller is a method to achieve a target state by applying corrections based on the following error metrics:

* Absolute Instantaneous Error (P) : Difference between current state and target state
* Instantaneous Change in Error (D) : Instantaneous rate of change of Cross track Error
* Overall Accumulated Error (I) : Accumulated Error

### Coefficients
* Kp: Is the Proportional Correction coefficient. It is responsible for the Direction and magnitude of the correction. However due to the momentum principle in control problems, Just using Kp will result in overshoot of the target state.
* Ki: Is the Integral Correction Coefficient. It is responsible for the elimination of the residual steady-state error that occurs with a pure proportional controller while increasing the speed of achieving target state. The momentum principle can plague the I controller as well so it is also prone to overshoot and oscillation
* Kd: Is the Differential Correction Coefficient. It is reponsbile for Dampening of the oscillation by being sensitive to the rate of change of instantaneous error rather than just pure magnitude or direction. Simply put, Kd will delay the achievement of target state but will also dampen and suppress the overshoot and oscillations.

## Tuning Methodology
The bulk of the work I did for this project was in tuning the hyperparameters and developing the twiddle algorithm to automatically update them as the car drove around the track.

I started out using some values supplied in the lecture:

```
P (Proportional) = 0.225
I (Integral) = 0.0004
D (Differential) = 4
```
I noticed that changing the integral param result in the car wildly oscillating back and forth. The same for the proportional param -- small changes resulting in large oscillations and the car would often go off the track. The differential value could be changed quite a bit before seeing much change in the behaviour

After I found the absolute values of the Kp, Ki, Kd that could complete one lap at 10 mph, the speed was gradually increased till the car could not complete one lap. Since results where hard to come, I decided to apply the twiddle algorithm to update the parameters automatically. Code is available in `twiddle.cpp`. The fundamental concept of twiddle is to refine input parameters based on an empirical feedback mechanism. The algorithm boils down to:

* Record the error before running
* Change the parameter
* Let the system run
* Record the error again
* Compare the two and chose the one with less error

When twiddle in enabled in the Udacity car simulator (uncomment lines in `main.cpp`), it updates the PID hyperparameters directly, and has an immediate affect on the car's performance.

### Tolerance
Twiddle incorporates a tolerance value as the hyperparameters are tuned, so the algorithm will know when it's finished. After some tinkering, I ended up keeping the same 0.2 value as used in the lab.

