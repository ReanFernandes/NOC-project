function [X_mpc, U_mpc, timings] = mpcDirectMultipleShooting(Q, R, N, T, modelParams, Tf, intialPose, setPoint1, setPoint2, distVec, initOption)
% MPC simulation with Direct Multiple Shooting scheme. We take only first
% control input u*0 in every iteration of MPC. The time for changing
% setpoint and external disturbance is fixed to N_sim/3 and 2N_sim/3.
% input:
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%           N:   Number of control intervals per window
%           T:   Time horizon
% modelParams:   Model parameters
%          Tf:   Simulaiton time for MPC
%  intialPose:   Initial position of the arm and the pendulum
%   setPoint1:   Setpoint for swinging up phase
%   setPoint2:   Change to this setpoint for testing stability of MPC afterwards
%     distVec:   External disturbance proportional to change in theta2_dot
%  initOption:   Initialization options (simple warm-start or shift) 
% output:
%       X_mpc:   State trajectory of the simulation
%       U_mpc:   Control inputs as output of MPC controller
%     timings:   Time spent for solving open-loop problem in MPC iterations 

%% Get the formulated NLP
[solver, w0, lbw, ubw, lbg, ubg] = formulateNLPDirectMultipleShooting(Q, R, N, T, modelParams);
% Dimension of variables
nx = 4; % dimension of state variables
nu = 1; % dimension of control input

h = T/N; % Timestep

%% Run the MPC Simulation
% Only apply first control input
N_sim = Tf / h;
% Initial pose of the system
x0 = intialPose;
setPoint = setPoint1;

w_opt = zeros(size(w0));

X_mpc = zeros(nx,N_sim);
U_mpc = zeros(nu,N_sim);
X_mpc(:,1) = x0;

timings = zeros(N_sim,1);

for i=1:N_sim
    tic;
    switch initOption
        case 1 % simple warm-start initialization, repeat w0 only
            w0 = w_opt;
        case 2 % shift initialization, shift w0 only, new value assign as zeros
            w0 = [w_opt((nx+nu)+1:end); zeros(nx+nu, 1)];
    end
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', [X_mpc(:,i); setPoint]);
    w_opt = full(sol.x);
    % Because we only apply u0*
    U_mpc(i) = w_opt(5);
    X_mpc(:,i+1) = w_opt(6:9);
    if (i == floor(N_sim/3))
        % Changing set point
        setPoint = setPoint2;
    end
    if (i == floor(2*N_sim/3))
        % External disturbance, e.g. pendulum is abruptly disturbed externally 
        % and has initial angular velocity distVec afterwards 
        X_mpc(4,i+1) = distVec;
    end
    % Save timing for current iteration
    timings(i) = toc;
end
end

