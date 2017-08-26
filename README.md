# CarND-Term2-P5-MPC
Model Predictive Control - Self-Driving Car Engineer Nanodegree Program

The goal of the project is to design an algorithm that will steer a car around a race track. The algorithm works as follows:
* (a) find the desired car route by fitting a polynomial into known way points ahead of the car (= desired route)
* (b) find steering angle, acceleration and brake parameters (= actuator values) which will minimize the driving error vs. desired route

The actuator values are updated every 100 ms, i.e. the above algorithm loops with a 100 ms interval.

## The Model

The state of the vehicle is modeled using 6 parameters:
* vehicle's position (x coordinate)
* vehicle's position (y coordinate)
* vehicle's orientation within the coordinate system (psi angle)
* vehicle's speed (v)
* distance from the desired position (cte = cross-track error)
* orientation error vs. desired orientation (epsi = psi error)

The actuators are modeled in the following way:
* steering angle: between -25 and 25 degrees
* acceleration / brake: between -1 and 1, where -1 equals full brake and 1 equals full acceleration

The model is based on the following state update equations:
* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

## Timestep Length and Elapsed Duration (N & dt)

I settled for N = 10 and dt = 0.1s, i.e. in each iteration the model predicts 10 car states up to 1s (10 * 0.1s) from the current point in time. I chose 1s as the prediction horizon partly based on the recommendation from the class (the horizon should be up to few seconds in the most extreme cases) and partly based on my own trial & error approach. Values higher than 1s did not provide smooth driving results.

For the N and dt values, I tried different combinations that would satisfy the equation N * dt == 1s. For instance:
* N = 5 and dt = 0.2s resulted in erratic driving and car veering off the track
* N = 20 and dt = 0.05s resulted in slightly less erratic driving, but the car still veered off the track at some point
Finally, I settled at N = 10 and dt = 0.1s. With these parameters the car drives smoothly.
Nevertheless, I must admit I'm quite surprised by the very high sensitivity of the model to the chosen N and dt parameters. I would like to understand the reasons for such a high sensitivity in more detail when I have some more spare time.

## Polynomial Fitting and MPC Preprocessing

A 3rd degree polynomial is fitted to waypoints. Prior to the polynomial fitting, the waypoints are transformed from the map's coordinate system into the vehicle's coordinate system. The transformation procedure covers centering coordinates around the car's position (x = 0, y =0) and rotating the view counter-clockwise by -psi.

Thanks to the above transformation, the calculation of the cross-track error and psi error is significantly simplified - the detailed reasoning is provided as comments in the code (int main() function):
* double cte = polyeval(coeffs, 0) - 0;
* double epsi = 0 - atan(coeffs[1]);

## Model Predictive Control with Latency

The latency is modeled by predicting the vehicle's state 100 ms into the future and feeding this future state into the MPC controller. The following lines of code from int main() model the latency:

          // Adjusting for 100 ms latency
          // Recall the equations for the model:
      	  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          //In the simulator however, a positive value implies a right turn and a negative value implies a left turn.
      	  //Therefore, we need to replace the "+" with the "-" sign in the equation below
          psi -= v / Lf * current_delta * latency;
          v += current_a * latency;
