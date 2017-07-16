# PID Controller writeup
Self-Driving Car Engineer Nanodegree Program

---

## Rubric Requirements

#### Compilation
- *Your code should compile* 
	* Yes, it does :-) 	

#### Implementation
- *The PID procedure follows what was taught in the lessons* 

As explained in the lesson, the implemented PID controller 

1. reads in the cross track error measurement `cte`, *i.e.*, distance from the lane center
2. updates its internal errors, calculating the derivative of `cte` as the difference between the previous and the current value and the integral as the summation of all previously observed errors
3. The PID controller output is minus the sum of these the three P, I, D errors multiplied by their respective coefficients

The code for the PID class can be found in `PID.cpp` and `PID.h`

#### Reflection
- *Describe the effect each of the P, I, D components had in your implementation* 

	* **P**: the proportional controller acts by steering the car with magnitude proportional to the `cte` and opposite in sign. Larger values correspond in larger steering angles, with the insurgence of more abrupt manoeuvers and large oscillations around the lane center. Too small values do not allow the car to steer enough to follow the road on a sharp turn.
	* **I**: the integral controller gradually eliminates the effect of systematic measurement errors. I did not observe any qualitative effect in the simulations, so it seems that the simulation environment does not suffer from systematic errors.
	* **D**: the strength of the derivative controller is proportional to the time derivative of the `cte`. Its effect is very visible, as it effectively prevents the car to overshoot the trajectory by damping the steering angle as it approaches the desired position of the car on the track.

- *Describe how the final hyperparameters were chosen.*

Here I would really like to say that the implemented twiddle algorithm (see in `PID.cpp`) allowed me to automatically tune the PID controller to find the best parameters. Reality is a bit different though: twiddle is a local hill climber, but in our case the hill has a strongly stochastic component as the optimisation progresses: as a metric to optimize, I chose the cumulative sum of absolute values of `cte` over a fixed time interval, but this is much more dependent on which portion of the track does was driven during that interval than on the parameters of the PID controller. The only clear exception is when the PID parameters move so far away from the ideal values that the car drives off the road. The only way to use twiddle effectively would be to drive the same portion of the track with each parameter choice, but this seems to be out of reach with the current code setup.

What I did instead was to start with the values `P=0.20, I=0.004, D=3.0` presented in the lessons as a hopefully meaningful initial guess, let the car drive around the track with the twiddle algorithm on and observe if and how the driving changed as the parameters were varying. It seems that the values `P=0.16, I=0.004, D=3.4` are appropriate for the throttle controller implemented (line 84 `main.cpp`).

#### Simulation
- *The vehicle must successfully drive a lap around the track*
	* It does drive around the track indefinitely, although I would probably turn down the offer of sitting on a SDC driving like that if I were asked. The driving is a bit jerky and braking only occurs *during* steering instead of anticipating it.
