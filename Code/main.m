clc
close all
clear variables
import casadi.*

%% Setup for plot
set(0,'defaultFigureRenderer', 'painters');  % For eps exporting
set(0,'defaultAxesFontSize', 10);            % Axis tick labels
set(0,'defaultTextFontSize', 14);            % Title, label
set(0,'defaultTextFontWeight','bold');       % Title, label
set(0,'defaultLineLineWidth', 2);            % Normal plot
set(0,'defaultStairLineWidth', 2);           % Stairs plot
savePath = 'Figures/';                       % Save location for figures

%% Model parameters
m2 = 0.3;
L1 = 0.2;
L2 = 0.3;
I1 = 0.0050; 
I2= 0.0025;
g = 9.81;

modelParams = struct('m2', m2, 'L1', L1, 'L2', L2, 'I1', I1, 'I2', I2, 'g', g);

%% Control parameters
% For control window and simulation time
Tf = 15; % Simulation time for MPC
T = 1.5; % Time per horizon
N = 40;  % Number of control intervals per horizon
h = T/N; % Timestep

% Dimension of variables
nx = 4; % dimension of state variables
nu = 1; % dimension of control input
% Order of polynomial for colocation
d = 3;
% Matrix for cost function L = (x-setPoint)^T*Q*(x-setPoint) + u^T*R*u
Q = zeros(nx, nx);
R = eye(nu);
% Upper bound for control input
u_max = 3;

% Initial pose of the pendulum (not every intial pose could lead to 
% reasonable solution)
initialPose = [0; pi; 0; 0];

%% Open loop
% Setpoint for open-loop control
setPoint = [0; 0; 0; 0];
% Test open-loop controller for the system
fprintf('Open-loop control for the system \n');
% ODE and Direct Multiple Shooting
fprintf('Open-loop control for the system with ODE and Direct Multiple Shooting... \n');
[X_odeOL, u_odeOL] = openLoopODE(Q, R, u_max, N, T, initialPose, setPoint, modelParams);
% DAE and Direct Collocation
fprintf('Open-loop control for the system with DAE and Direct Collocation... \n');
[X_daeOL, u_daeOL] = openLoopDAE(d, Q, R, u_max, N, T, initialPose, setPoint, modelParams);

% Plot
fig = figure();
% Plot resolution, depends on type of plot
fig.Position = [10 10 800 600];
plot2Trajectory(X_odeOL, u_odeOL, h, 'Direct Multiple Shooting', X_daeOL, u_daeOL, h, 'Direct Collocaiton', u_max);
% Save figure
saveFigure(fig, savePath, 'OpenLoopControl');

% Animate the pendulum (optional, take long time)
% animateIP(X_odeOL, h, L1, L2, savePath, 'OpenLoopMultipleShooting.gif');
% animateIP(X_daeOL, h, L1, L2, savePath, 'OpenLoopDirectCollocation.gif');
fprintf('Done executing open-loop control \n\n');

%% MPC Simulation
% Test MPC by swinging up the pendulum, changing setpoint and applying an
% external pulse (i.e. abruptly change in angular velociy of pendulum)

% Setpoint when pendulum is stable (i.e. finishing swinging up)
setPoint1 = [pi; 0; 0; 0];
% Change to this setpoint to test stability
setPoint2 = [pi/2; 0; 0; 0];
% Disturbance proportional to sudden change in velocity to test stability
distVec = 2*pi;

fprintf('Test MPC with 3 different schemes \n');
% MPC with 3 different schemes, not use initialization for MPC iterations
% MPC with Direct Multiple Shooting
fprintf('Test MPC with Direct Multiple Shooting... \n');
[X_dmsMPC, U_dmsMPC] = mpcDirectMultipleShooting(Q, R, u_max, N, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, 0);

% MPC with Direct Collocation
fprintf('Test MPC with Direct Collocation... \n');
[X_dcMPC, U_dcMPC] = mpcDirectCollocation(d, Q, R, u_max, N, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, 0);

% MPC with RTI
fprintf('Test MPC with Real Time Iteration... \n');
[X_rtiMPC, U_rtiMPC] = mpcRealTimeIteration(Q, R, u_max, N, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec);

% Plot
fig = figure();
% Plot resolution, depends on type of plot
fig.Position = [10 10 1200 720];
plot3Trajectory(X_dmsMPC, U_dmsMPC, h, 'Direct Multiple Shooting', X_dcMPC, U_dcMPC, h, 'Direct Collocation', ...
    X_rtiMPC, U_rtiMPC, h, 'Real Time Iteration', u_max);
% Save figure
saveFigure(fig, savePath, 'Test3MpcSchemes');

% Animate the pendulum (optional, take long time)
% animateIP(X_dmsMPC, h, L1, L2, savePath, 'MpcMultipleShooting.gif');
% animateIP(X_dcMPC, h, L1, L2, savePath, 'MpcDirectCollocation.gif');
% animateIP(X_rtiMPC, h, L1, L2, savePath, 'MpcRTI.gif');
fprintf('Done testing different MPC schemes \n\n');

%% Test runtime with different configurations
% Test runtime of 3 methods with fixed setpoints and disturbance velocity.
% Initialization method for MPC are: none, warm-start, shift. RTI does not
% need initialization method. N is set to 40, 30, 25. For N less than
% 40, RTI is not solvable.

fprintf('Test runtime with different configurations \n');

N_config = [40; 30];

% For plotting and comparison between Direct Multpile Shooting and 
% Direct Collocation

X_test_plot_1 = [];
U_test_plot_1 = [];

X_test_plot_2 = [];
U_test_plot_2 = [];

h_test_plot_1 = 0;
h_test_plot_2 = 0;

for i=1:length(N_config)
    N_test = N_config(i);
    if (N_test >= 40)
        for j=0:2
            [X_test_dms, U_test_dms, timings_test_dms] = ...
            mpcDirectMultipleShooting(Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, j);
            timeDMS = sum(timings_test_dms);
            fprintf(['Window Length: N = %d; Initialization Option: %d; MPC Direct Multiple Shooting takes total %.3f seconds, '...
                'on average %.3f seconds per MPC iteration \n'], N_test, j, timeDMS, timeDMS/length(timings_test_dms));

            [X_test_dc, U_test_dc, timings_test_dc] = ...
            mpcDirectCollocation(d, Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, j);
            timeDC = sum(timings_test_dc);
            fprintf(['Window Length: N = %d; Initialization Option: %d, MPC Direct Collocation takes total %.3f seconds, ' ...
                'on average %.3f seconds per MPC iteration \n'], N_test, j, timeDC, timeDC/length(timings_test_dc));
            % Plot for initialization option 1
            if (j == 1)
                X_test_plot_1 = X_test_dc;
                U_test_plot_1 = U_test_dc;
                h_test_plot_1 = T/N_test;
            end

            [X_test_rti, U_test_rti, timings_test_rti] = ...
            mpcRealTimeIteration(Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec);
            timeRTI = sum(timings_test_rti);
            fprintf(['Window Length: N = %d, MPC Real Time Iteration takes total %.3f seconds, on average ' ...
                '%.3f seconds per MPC iteration \n'], N_test, timeRTI, timeRTI/length(timings_test_rti));
        end
    else
        for j=0:2
            [X_test_dms, U_test_dms, timings_test_dms] = ...
            mpcDirectMultipleShooting(Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, j);
            timeDMS = sum(timings_test_dms);
            fprintf(['Window Length: N = %d; Initialization Option: %d; MPC Direct Multiple Shooting takes total %.3f seconds, '...
                'on average %.3f seconds per MPC iteration \n'], N_test, j, timeDMS, timeDMS/length(timings_test_dms));

            [X_test_dc, U_test_dc, timings_test_dc] = ...
            mpcDirectCollocation(d, Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, j);
            timeDC = sum(timings_test_dc);
            fprintf(['Window Length: N = %d; Initialization Option: %d, MPC Direct Collocation takes total %.3f seconds, ' ...
                'on average %.3f seconds per MPC iteration \n'], N_test, j, timeDC, timeDC/length(timings_test_dc));
            
            [X_test_rti, U_test_rti, timings_test_rti] = ...
            mpcRealTimeIteration(Q, R, u_max, N_test, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec);
            timeRTI = sum(timings_test_rti);
            fprintf(['Window Length: N = %d, MPC Real Time Iteration takes total %.3f seconds, on average ' ...
                '%.3f seconds per MPC iteration \n'], N_test, timeRTI, timeRTI/length(timings_test_rti));
            % Plot for initialization option 1
            if (j == 1)
                X_test_plot_2 = [X_test_plot_2; X_test_dms; X_test_rti];
                U_test_plot_2 = [U_test_plot_2; U_test_dms; U_test_rti];
                h_test_plot_2 = T/N_test;
            end
        end
    end
end

% Plot
fig = figure();
% Plot resolution, depends on type of plot
fig.Position = [10 10 1200 720];
plot3Trajectory(X_test_plot_1, U_test_plot_1, h_test_plot_1, 'Direct Collocation, N = 40', ...
    X_test_plot_2(1:4,:), U_test_plot_2(1,:), h_test_plot_2, 'Direct Multiple Shooting, N = 30', ...
    X_test_plot_2(5:8,:), U_test_plot_2(2,:), h_test_plot_2, 'Real Time Iteration, N = 30', u_max);
% Save figure
saveFigure(fig, savePath, 'TestRuntime');
fprintf('Done testing runtime \n\n');

%% Angular periodicity
N = 30;  % Number of control intervals per window
h = T/N; % Timestep
% Setpoint which is not in [-pi, pi]
setPoint1 = [5*pi/2; 0; 0; 0];
setPoint2 = [7*pi/2; 0; 0; 0];
% Matrix Q for augmented cost function
Q_aug = diag([0.03; 0.03; 0.03; 0.03; 0.005; 0.005]);

fprintf('Test MPC with angular periodicity \n');
% MPC which takes angular periodicity into account
[X_mpcPer, U_mpcPer, timings_mpcPer] = mpcPeriodicity(Q_aug, R, u_max, N, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, 2);
% Compare with MPC which does not take angular periodicity into account
[X_mpcNonPer, U_mpcNonPer, timings_mpcNonPer] = mpcDirectMultipleShooting(Q, R, u_max, N, T, modelParams, Tf, initialPose, setPoint1, setPoint2, distVec, 2);

% Print runtime
fprintf('Considering periodicity, Direct Multiple Shooting takes total %.3f seconds \n', sum(timings_mpcPer));
fprintf('Not considering periodicity, Direct Multiple Shooting takes total %.3f seconds \n', sum(timings_mpcNonPer));

% Plot
fig = figure();
% Plot resolution, depends on type of plot
fig.Position = [10 10 800 600];
plot2Trajectory(X_mpcPer, U_mpcPer, h, 'Considering Angular Periodicity', ...
    X_mpcNonPer, U_mpcNonPer, h, 'Not Considering Angular Periodicity', u_max);
% Save figure
saveFigure(fig, savePath, 'TestPeriodicity');

% Animate the pendulum (optional, take long time)
% animateIP(X_mpcPer, h, L1, L2, savePath, 'MpcAngularPeriodicity.gif');
% animateIP(X_mpcNonPer, h, L1, L2, savePath, 'MpcAngularNonPeriodicity.gif');
fprintf('Done testing MPC with angular periodicity \n\n');

%% Helper function for plot saving
function saveFigure(fig, savePath, plotName)
% Save plot
% input:
%        fig:  Figure which would be saved
%   savePath:  Location for saving
%   plotName:  Name of the plot

    if ~isfolder(savePath)
        mkdir(savePath);
    end
    % Save figure as .eps file
    filename = strcat(fullfile(savePath, plotName), '.eps');
    % exportgraphics() function only works with R2020a or newer versions
    exportgraphics(fig, filename, 'Resolution', 300);
end
