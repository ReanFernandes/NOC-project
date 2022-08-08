function [solver, w, lbw, ubw, lbg, ubg, f] = formulateNLPRealTimeIteration(Q, R, N, T, modelParams)
% Formulate NLP with Real-Time Iteration scheme, based on Exercise 10,
% Numerical Optimal Control Course SS2022.
% This function supports cost function L = (x-setPoint)^T*Q*(x-setPoint) + u^T*R*u
% input:
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%           N:   Number of control intervals per horizon
%           T:   Time horizon
% modelParams:   Model parameters
% output:
%      solver:   Solver which takes linearization point, initial state and setpoint as parameters
%           w:   Decision variables
%         lbw:   Lower bound vector for decision variables
%         ubw:   Upper bound vector for decision variables
%         lbg:   Lower bound for constraints
%         ubg:   Upper bound for constraints
%           f:   Integrator

import casadi.*

%% Set up necessary terms
% Dimension of variables
nx = 4; % dimension of state variables
nu = 1; % dimension of control input

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
w0 = [w0; 0; pi; 0; 0];
% Equality constraints
g = {g{:}, X0 - x0_hat};
lbg = [lbg; 0; 0; 0; 0];
ubg = [ubg; 0; 0; 0; 0];

Xk = x0_hat;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu, 1);
    w = [w(:)', {Uk}];
    lbw = [lbw; -3];
    ubw = [ubw;  3];
    w0 = [w0; 1];

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
    % Terminal constraint
    if (k == N - 1)
        g = [g, {x_setpoint-Xk}];
        lbg = [lbg; 0; 0; 0; 0];
        ubg = [ubg; 0; 0; 0; 0];
    end
end

% Create an NLP solver
w = vertcat(w{:});
g = vertcat(g{:});

G = Function('G', {w, x0_hat},{g}); % All constraints
JG = Function('JG', {w, x0_hat},{jacobian(g,w)});
% Linearize constraints
wk = MX.sym('wk',length(w),1); % linearization point
g_l = G(wk, x0_hat) + JG(wk, x0_hat)*(w - wk); % linearized constraints

H = kron(eye(N), diag([diag(Q);diag(R)]));
H = diag([diag(H);diag(zeros(nx))]);

J = 1/2*w.'*H*w;

p_in = [wk; x0_hat; x_setpoint]; % QP parameter
qp = struct('x', w, 'f', J, 'g', g_l, 'p', p_in); % QP struct
qp_opts = struct('printLevel', 'none');   % No printing of evaluations
solver = qpsol('solver', 'qpoases', qp, qp_opts); % Allocate QP solve
end

