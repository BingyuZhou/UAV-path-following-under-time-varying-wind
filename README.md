# UAV-path-following-under-time-varying-wind
A simulation of UAV path following stratagy under time varying wind.

Introduction:

An adaptive control scheme for Unmanned Aerial Vehicles (UAVs) path following under slowly time-varying wind is developed. 
The proposed control strategy integrates the path following law based on the vector field
method with an adaptive term counteracting the effect of wind’s
unknown component. The control strategy is developed and
numerically simulated for straight line and orbit following
scenarios. In particular, it is shown that the path following
error is bounded under slowly time-varying unknown wind
and converges to zero for unknown constant wind. Numerical
simulations illustrate that, in environments with unknown and
slowly time-varying wind conditions, the proposed method
compensates for the lack of knowledge of the wind vector,
and attains a smaller path following error than state-of-theart vector field method.

Use of code:

Two methods are implemented in the Matlab/Simulink platform. 

#1. Standard vector field method:

The main file is BeardMethod.m. This file also calls the
OrbitFollowing.slx and StraightLine.slx. For people who wants to
know more about the method, plese read the paper: 

D. R. Nelson, D. B. Barber, T. W. McLain, and R. W. Beard, “Vector
field path following for miniature air vehicles,” IEEE Transactions on
Robotics, vol. 23, no. 3, pp. 519–529, June 2007.

#2. Adaptive VF:

This is main contribution of our team. The main file is RevisedMethod.m, which calls the RevisedOrbitFollowing.slx and RevisedStraightLine.slx.

Another variant is named RevisedOrbitFollowing2.slx and RevisedStraightLine2.slx.

You can use this variant if you change the simulink file to RevisedOrbitFollowing2.slx and RevisedStraightLine2.slx in RevisedMethod.m.

The only difference between these two variants are the implementations of the wind disturbance.

