# Numerical Optimal Control (SS 2022) project

We implement a Model Predictive Control (MPC) approach using Direct Multiple Shooting, Direct Collocation and Real-Time Iteration for a Furuta Pendulum.

## Requirements

* [MATLAB](https://www.mathworks.com/products/matlab.html) (version R2020a or newer)
* [CasADi](https://web.casadi.org/)

## Description

* main.m: main file
* modelODE.m: system model in ODE form
* modelDAE.m: system model in DAE form
* rk4Integrator.m: single-step explicit RK4 numerical integrator
* formulateNLPDirectMultipleShooting.m: formulate NLP with Direct Multiple Shooting and ODE
* formulateNLPDirectCollocation.m: formulate NLP with Direct Collocation and DAE
* formulateNLPRealTimeIteration.m: formulate NLP with Real-Time Iteration and ODE
* formulateNLPPeriodicity.m: formulate NLP considering angular periodicity
* openLoopODE.m: open-loop with Direct Multiple Shooting and ODE
* openLoopDAE.m: open-loop with Direct Collocation and DAE
* mpcDirectMultipleShooting.m: MPC simulation with Direct Multiple Shooting
* mpcDirectCollocation.m: MPC simulation with Direct Collocaiton
* mpcRealTimeIteration.m: MPC simulation with Real-Time Iteration
* mpcPeriodicity.m: MPC simulation considering angular periodicity
* plot2Trajectory.m: plot 2 state trajectories side by side
* plot3Trajectory.m: plot 3 state trajectories side by side
* animiateIP.m: animate inverted pendulum with a given trajectory input

## Acknowledgements

This code was partially based in the direct_multiple_shooting.m and direct_collocation.m example from CasADi.

We would like to thank Florian Messerer and Prof. Dr. Moritz Diehl for the supervision in this project.
