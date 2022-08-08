function [X_odeOL, U_odeOL] = openLoopODE(Q, R, N, T, modelParams)
% Open-loop with Direct Multiple Shooting and ODE
% input:
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%           N:   Number of control intervals per window
%           T:   Time horizon
% modelParams:   Model parameters
% output:
%     X_odeOL:   State trajectory
%     U_odeOL:   Control inputs sequence of the open-loop controller

import casadi.*

%% Get the formulated NLP
[solver, w0, lbw, ubw, lbg, ubg] = formulateNLPDirectMultipleShooting(Q, R, N, T, modelParams);

%% Solve the NLP
x0 = [0; pi; 0; 0];
setPoint = [0; 0; 0; 0];

sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', [x0; setPoint]);
w_opt = full(sol.x);
x1_opt = w_opt(1:5:end);
x2_opt = w_opt(2:5:end);
x3_opt = w_opt(3:5:end);
x4_opt = w_opt(4:5:end);

X_odeOL = vertcat(x1_opt', x2_opt', x3_opt', x4_opt');
U_odeOL = w_opt(5:5:end)';
end

