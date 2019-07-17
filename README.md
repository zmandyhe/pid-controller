# PID Controller
Self-Driving Car Engineer Nanodegree Program

## Goal
This project is to implement a PID controller in C++ to maneuver the vehicle around the track in Udacity's simulator. The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

PID stands for Proportional-Integral-Deifferential. These three controllers are combined in such a way that it produces a control signal. This is how the vehicle uses steering, throttle, and brake to move through the world, executing a trajectory created by the path planning block.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Reflections

### Describe the effect each of the P, I, D components had in your implementation.
*P Controller:* 
Propotional to the cross track error, it is to help the car to stay propotionally with the track. However, it could result overshooting the track. So it will need D controller to adjust the overshooting issue.
*I Controller:* 
Integral, it is to sum up the system error to compensate for biases. With the integral term we're keeping track of all the previous CTEs,initially we set int_cte to 0 and then add the current cte term to the count int_cte += cte. Finally we update the steering value, -tau_p * cte - tau_d * diff_cte - tau_i * int_cte with the new tau_i parameter.
*D Controller:*
Differential, the difference between the current CTE and previous CTE. It helps to counteracts the overshooting issue caused by P controller. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

The controller worked as exptectd when I adjusted their parameter values to finally stablize the car to stay on the track even on the sharp turn road. As shown in the following PID parameters, I increased the P controller parameter from 0.13 to 0.33 that is able to stay propotional to the track in the sharp left and right turn; at the same time, I increased the differential controller value from 3.05 to 4.05, it counteracts the overshooting on the sharp turns that it enables my car to stay on the center of the road. I set a relatively small system error for the I controller which helped stablized the car from system noises. Below shows the final parameters setting of PID controller.
```
pid_s.Init(0.33, 0.0027, 4.05);
```
![alt adjust overshotting](https://github.com/zmandyhe/pid-controller/blob/master/left-turn.png)

### Describe how the final hyperparameters were chosen.

I implemented Twittle algorithm to find the optimized PID controller parameters. The Twittle algorithm take 1000 steps to optimize the p values. After the steps, the car is able to drive on the track for most time, but overshoot off the road on the sharp turns. To solve this problem I manually tuned the parameters of PID controllers with a higher propotional value and a higher differential value to force the car to stay propotionally with the track and at the same time be able to adjust the overshooting problems on sharp turns to steer sharply so as to stay on the center of the track. It works well.

Thottle parameter value are manually tuned to work with PID controller in controlling the speed and steering angles.

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

## Code Style

Refer to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
