# PID Controller Project
Self-Driving Car Engineer Nanodegree Program

## Project Description
The purpose of this project is to build a PID controller. The simulator provides cross-track error (CTE), speed, and steering angle data via local websocket. The PID (proportional/integral/differential) controller must respond with steering and throttle commands to drive the car reliably around the simulator track.

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

## Basic Instructions

1. Clone this repo.
2. Update initial values in main.cpp if you want
   1. Line 50: initial p value
   2. Line 51: initial dp value
   3. Line 68: the number of data point used per cycle
   4. Line 69: tolerance
   5. Line 191: throttle
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it for Twiddle: `./pid twiddle`
6. Run the simulator
7. Once the optimal p value is found, update Line 75 in main.cpp
8. Run it for simulation: `./pid`
9. Run the simulator

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

---

## Reflection

### PID Control
PID stands for "P"roportional, "I"ntegral, "D"erivative. It uses three terms to control the vehicle to reduce the cross tracking error (CTE). A PID controller continuously calculates an error value as the difference between a desired set point and a measured process variable, and applies a correction based on proportional, integral, and derivative terms as:

<img src="img/pid_formula.png" width="480" />

Term P is a proportional component to the current value of CTE. This makes the car to steer proportional to the car's distance from the lane center (i.e. CTE) - if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

Term D is a derivative component of CTE, which makes the car overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

Term I is an integral component of CTE, which corrects a bias in the CTE which prevents the P-D controller from reaching the center line. This can minimise the steering drift or mechanical errors by reducing the total accumulated error, and in this project it helps to reduce the error around curves.

### Parameter Tuning
At first, I needed to find the initial parameter of Kp, Ki and Kd which can at least drive the car on the track.
This was done by manually changing those initial values many times.
Started from Kp=0.1, Ki=0.1 and Kd =0.1, which put the car off the road right after the start.
Changing Kp didn't help much.
Then, gradually increased Kd, which could keep the car in the track at least till the first corner where the car was off the road.
Finally, decreased Ki could keep the car in the track for a while.
Thus, I chose the initial p value as {0.1, 0.01, 2}

Next, I used Twiddle algorithm to find the optimal p value.
First I set the initial dp = p but it actually too large as added or subtracted from p.
Then I chose one tenth of p as initial dp value - i.e. dp = {0.01, 0.001, 0.2}, which could keep the car within the road.

For one cycle, considering to differentiate from other students' result and also include various turns in the course, I decided to use throttle value of 0.6 and number of data points of 1,000.
Also, I set the tolerance value as 0.1 for sum of dp. In fact, it has never reached to the point even after running a couple of hours.
So, I stopped at cycle #10 of entire twiddle process.
The final parameter I got from above process was p = {0.109, 0.001, 2.6378} with the error as 0.219123

It looked unstable even in the later cycles and I think keeping the throttle as 0.6 all the time is too fast to control in this set. As a future work, I will work on applying similar controller to the throttle as well as steering.

### Simulation
Finally, I ran the simulator to confirm the vehicle can drive a round without going off track with the parameter of p = {0.109, 0.001, 2.6378}
<img src="img/simulator.png" width="640" />