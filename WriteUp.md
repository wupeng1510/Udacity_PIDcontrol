# Udacity_PIDcontrol
Project of PID control in udacity

This writeup is for the PID controller implementation in CPP.

Ⅰ Introduction

PID stands for Proportional-Integral-Derivative. These three components are combined in such a way that it produces a control signal. Before getting into the implementation, let’s discuss the PID controller components. I am going to discuss based on how to use PID for steering angles.

1. Cross Track Error
A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

2. P component
It sets the steering angle in proportion to CTE with a proportional factor tau.
-tau_p * cte
In other words, the P, or "proportional", component had the most directly observable effect on the car’s behaviour. It causes the car to steer proportional (and opposite) to the car’s distance from the lane center(CTE) - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right.

3. D component
It’s the differential component of the controller which helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis.

In other words, the D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.
diff_cte = cte - pre_cte
pre_cte = cte
-tau_d * diff_cte

4. I component
It’s the integral or sum of error to deal with systematic biases.

In other words, the I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift , but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

int_cte +=cte
-tau_i * int_cte

And combination of these we can get PID controller to control the steering value.

steer_value = -tau_p * cte -tau_d * diff_cte - tau_i * int_cte

Parameter optimisation can be done manually or using Twiddle algorithm.

Ⅱ PID controller implementation

1. Finding initial value for Kp, Ki, Kd(tau_p, tau_i, tau_d)
The intial value for Kp, Ki, Kd selected using trail and error method. It is a simple method of PID controller tuning. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches to oscillating behaviour. Then Kd was tuned to reduced oscillation and then Ki to reduce steady-state error.

Car started to oscillate when KP value set to 0.1 whereas Ki and Kd set to zero.

Then found the Kd value that stops the oscillating behaviour which is set to 2.0 along with 0.05 for Kp and zero for Ki.

Finally value Ki set as 0.0001 to reduce the steady-state error.

Initial value for Kp, Ki, and Kd set as below:
(0.1, 0.0001, 2.0)

2. Twiddle
Once we set the initial value, Twiddle algorithm is used to optimise the parameters.

I modified the main.cpp to implement Twiddle algorithm. When twiddle variable set to true, simulator runs the car with confidents till the maximum steps set initially and go through the twiddle algorithm. After competition of each iteration, simulator reset to initial stage and car runs starts from the beginning to maximum steps. This process continuous until tol value below the allowed value.

Optimised value we got as below:
(0.246379,0.000198217,2.02249)
Once I found the optimised value, set the twiddle variable to false to run the car through the simulator.

bool twiddle = false;

Ⅲ discussion
1. The low speed at the first few seconds and the large curvature in training of twiddle can make measurement of error inaccuracy.
In other word, the road of the training twiddle, can influent the parameter, and the performance of running a loop.

2. The method of how to check if the twiddle is done is need to be improved,because if the biggest parameter restrain at first,and reach the tolerance, the twiddle may be done by mistake.
dp[0] + dp[1] + dp[2] < tolerance 
For example, tolerance = 0.1, dp[0]=0.01, dp[1]=0.00001,dp[2]=0.2
if the dp[2] restain at the earliest time, and decrease under 0.1, the twiddle need go on, instead of done.
I suppose the method below ,and because of the limited GPU time, I havnt have a try.
dp[0]/p[0] < 0.001 && dp[1]/p[1] < 0.001 && dp[2]/p[2] < 0.001 

3. The basic mothod of twiddle is ineffective and need to be improved .
For example it may search the area repeatedly, and it may jump the parameters that already get a good tolerance.
