function [solver, w0, lbw, ubw, lbg, ubg] = formulateNLPDirectMultipleShooting(Q, R, u_max, N, T, modelParams)
% Formulate NLP with Direct Multiple Shooting method.
% This function is based on CasADi example direct_multiple_shooting.m.
% This function supports cost function L = (x-setPoint)^T*Q*(x-setPoint) + u^T*R*u
% with constraint -u_max < u < m_max. If Q == zero(nx,nx), terminal
% constraint would be activated
% input:
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%       u_max:   Upper bound for control input
%           N:   Number of control intervals per horizon
%           T:   Time horizon
% modelParams:   Model parameters
% output:
%      solver:   Solver which takes initial state and setpoint as parameters
%          w0:   Initial guess for decision variables, all zeros
%         lbw:   Lower bound vector for decision variables
%         ubw:   Upper bound vector for decision variables
%         lbg:   Lower bound for constraints
%         ubg:   Upper bound for constraints

import casadi.*

%% Set up necessary terms
% Dimension of variables
nx = 4; % Dimension of state variables
nu = 1; % Dimension of control input

h = T/N; % Timestep

% State variable [theta1, theta2, theta1_dot, theta2_dot]
x = MX.sym('x', nx, 1);
% Control input
u = MX.sym('u', nu, 1);

% Model equation
xdot = modelODE(x, u, modelParams);

% Objective term 
x_setpoint = MX.sym('x_setpoint', nx, 1);
L = (x-x_setpoint)'*Q*(x-x_setpoint) + u'*R*u;

% Integrator
f = rk4Integrator(x, u, xdot, L, h);

%% Build up NLP problem
% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% Multiple shooting
% Formulate the NLP
x0_hat = MX.sym('x0_hat', nx, 1);

X0 = MX.sym('X0', nx, 1);
w = {w{:}, X0};
% Constraints for x0
lbw = [lbw; -inf; -inf; -inf; -inf];
ubw = [ubw;  inf;  inf; inf; inf];
% Initial guess
w0 = [w0; 0; 0; 0; 0];
% Equality constraints
g = {g{:}, X0 - x0_hat};
lbg = [lbg; 0; 0; 0; 0];
ubg = [ubg; 0; 0; 0; 0];

Xk = x0_hat;

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu, 1);
    w = [w(:)', {Uk}];
    lbw = [lbw; -u_max];
    ubw = [ubw;  u_max];
    w0 = [w0; 0];

    % Integrate till the end of the interval
    Fk = f('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = [w(:)', {Xk}];
    lbw = [lbw; -inf; -inf; -inf; -inf];
    ubw = [ubw;  inf;  inf; inf; inf];
    w0 = [w0; 0; 0; 0; 0];

    % Add equality constraint
    g = [g, {Xk_end-Xk}];
    lbg = [lbg; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0];
end
% Terminal constraint
if (Q == zeros(nx, nx))
    g = [g, {x_setpoint-Xk}];
    lbg = [lbg; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0];
end

% Create an NLP solver
w = vertcat(w{:});
g = vertcat(g{:});
prob = struct('f', J, 'x', w, 'g', g, 'p', [x0_hat; x_setpoint]);
nlpsol_opts = struct;
nlpsol_opts.ipopt.print_level=0;    % No printing of evaluations
nlpsol_opts.print_time= 0;          % No printing of time
solver = nlpsol('solver', 'ipopt', prob, nlpsol_opts);
end

