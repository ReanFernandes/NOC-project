function [ F ] = rk4Integrator(x, u, xdot, L, timeStep)
% RK4 integrator for ODE. This is based on CasADi example
% "direct_multiple_shooting.m"
% input:
%       x:   initial state variable for this step
%       u:   control input
%    xdot:   ODE model
%       L:   stage cost function
%timeStep:   timestep
% output:
%    xdot:   differentiation of state variables

import casadi.*

M_RK4 = 1; % RK4 steps per interval
DT = timeStep/M_RK4;
f = Function('f', {x, u}, {xdot, L});
X0 = MX.sym('X0', size(x));
U = MX.sym('U', size(u));
X = X0;
Q = 0;
for j=1:M_RK4
   [k1, k1_q] = f(X, U);
   [k2, k2_q] = f(X + DT/2 * k1, U);
   [k3, k3_q] = f(X + DT/2 * k2, U);
   [k4, k4_q] = f(X + DT * k3, U);
   X=X + DT/6*(k1 + 2*k2 + 2*k3 + k4);
   Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

end
