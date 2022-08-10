function [X_daeOL, U_daeOL] = openLoopDAE(d, Q, R, u_max, N, T, initialPose, setPoint, modelParams)
% Open-loop with Direct Collocaiton and DAE
% input:
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%       u_max:   Upper bound for control input
%           N:   Number of control intervals per window
%           T:   Time horizon
% initialPose:   Initial pose of the pendulum
%    setPoint:   Setpoint for open-loop control
% modelParams:   Model parameters
% output:
%     X_daeOL:   State trajectory
%     U_daeOL:   Control inputs sequence of the open-loop controller

import casadi.*

%% Get the formulated NLP
[solver, w0, lbw, ubw, lbg, ubg] = formulateNLPDirectCollocation(d, Q, R, u_max, N, T, modelParams);

%% Solve the NLP
x0 = initialPose;

sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', [x0; setPoint]);
w_opt = full(sol.x);
% x has dimension 4x1, algebraic variable z has dimension 2x1
x1_opt = w_opt(1:5+6*d:end);
x2_opt = w_opt(2:5+6*d:end);
x3_opt = w_opt(3:5+6*d:end);
x4_opt = w_opt(4:5+6*d:end);

X_daeOL = vertcat(x1_opt', x2_opt', x3_opt', x4_opt');
U_daeOL = w_opt(5:5+6*d:end)';
end

