# Reflections for CarND-Controls-PID 
Self-Driving Car Engineer Nanodegree Program

## Approach

All video files referenced below are in the videos folder.

### Without twiddle, finding out good initial values for Kp, Ki and Kd

I know that the road ledge corresponds to about 5 cte (cross-track-error). The vehicle 
should never approach the ledge. Steer values should be [-1, 1]. Thus, a good 
starting value for Kp is 1/5 where the biggest steering value is output at cte of 5.

The result is oscillatory behaviour with ever increasing amplitudes, which is expected.
Watch the video file at pid_p_0.2_i_0_d_0.mov.

Increasing the Kp to 0.5 results in higher frequency and larger amplitudes.
Watch the video file at pid_p_0.5_i_0_d_0.mov.

Next, Kd acts as a damping force to an otherwise undamped oscillation. Setting Kd to 
0.5 reduces the amplitude of oscillation, the car is able to get further,
but it is still too small to prevent it from hitting the ledge.
Watch the video file at pid_p_0.2_i_0_d_0.5.mov.

Increasing Kd to 5 results in less overshoot. If the car veers off center, the p term
steers it towards the middle, as it approaches the middle, cte decreases, the steering
angle decreases to the point where there is little to no overshoot.
Watch the video file at pid_p_0.2_i_0_d_5.mov.

Finally, Ki offsets any long term bias or dift in the cte. In the previous videos,   
notice that the car drifts off center in a curve? That is with Ki set to 0.
Setting Ki to a large value like 0.05 causes erratic behaviour as it becomes the
dominant term in the controller.
See the video file at pid_p_0.2_i_0.05_d_5.mov.

Setting Ki to a reasonable value like 0.005 allows the car to keep to the center
while turning.
See the video file at pid_p_0.2_i_0.005_d_5.mov.

### Initialize using the values found above, optimize using Twiddle

With initial Kp = 0.2, Ki = 0.005, Kd = 5, dp = 0.02, di = 0.0005, dd = 0.5,
I ran the twiddle algorithm for optimizing hyperparameters until sum of dp, di, dd <
tol = 0.05. 

The final values are: Kp = 0.141, Ki = 0.00263, Kd = 4.80.

The final values for differentials are: dp = 0.0129, di = 0.000263, dd = 0.0354.

If I kept the twiddle algorithm running, the parameters would eventually change to
the point where the controller cannot handle curves and will drive off the track.
A tol value of 0.05 is a good stopping point for this track.


[Watch PID with twiddle youtube video here![watch video here](https://img.youtube.com/vi/w0rLFDO_GqY/0.jpg)](https://youtu.be/w0rLFDO_GqY)


### Fastest lap by controlling throttle (speed)

I designed my own PID controller for throttle based on the formula below.

throttle = bias + steer_factor + p - d + i

bias = constant term

steer_factor = A * (1 - |steer_value|), if driving straight should go fast, if turning, should slow down

p = B * |cte| - speed should be proportional to cte, assuming steer_value is correct, the further away 
you are from the trajectory, the faster you should correct yourself

d = C * (delta cte), differential term. If cte increases (at a turn for e.g.) should slow down, 
if cte decreases (straight road) should speed up

i = D * (longTermAverageError - fmin(avg_error, E)), integral term. If current cte average over last 50 time steps is low, 
I should speed up. If current cte average is high, slow down. 
fmin(avg_error, E) is to put a ceiling value of E, to prevent cases of extreme high error at the beginning from setting throttle to 0.

Hyperparameters A, B, C, D, E are tweaked by hand, longTermAverageError from observations.

Final values for A = 0.18, B = 0.5, C = 0.2, D = 2.5, E = 0.4

[Watch PID with steer and throttle PID video here![watch video here](https://img.youtube.com/vi/eYKSPCYi4fA/0.jpg)](https://youtu.be/eYKSPCYi4fA)


## Implementation tips

1. Be careful of signs. My final steering value is calculated using value = p - i - d.
As cte can be (+ve) or (-ve), what you want is for Kp, Ki and Kd to reflect the magnitude, 
the sign/direction is captured by cte and the fomula. Thus, when modifying the params during twiddle,
always take the absolute value after twiddling.
2. Set the window size to take average cte over to be big enough to be resistant to noise, but small enough to
be responsive to terrain changes. Changing from 100 to 50 helps the car to recognize quickly that it is driving
on a straight road and speed up, improving its fastest lap time. 
3. Set bounds on steer and throttle using fmax() and fmin() to prevent erratic and unstable behaviour.

