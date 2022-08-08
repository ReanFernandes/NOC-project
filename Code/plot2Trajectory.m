function [] = plot2Trajectory(X1, U1, h1, pTitle1, X2, U2, h2, pTitle2)
% Plot 2 trajectories of the pendulum side by side
% input:
%       X1:  Horizontally stacked vectors of state of trajectory 1
%       U1:  Horizontally stacked vectors of control input of trajectory 1
%       h1:  Time step for trajectory 1
%  pTitle1:  Title for the plot of trajectory 1
%       X2:  Horizontally stacked vectors of state of trajectory 2
%       U2:  Horizontally stacked vectors of control input of trajectory 2
%       h2:  Time step for trajectory 2
%  pTitle2:  Title for the plot of trajectory 2

N1 = length(U1);
subplot(321);
hold all;
plot([0:N1]*h1, X1(1,:), '-');
plot([0:N1]*h1, X1(2,:), '-');
ylabel('Angular Position (rad)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\theta_1$','$\theta_2$', 'Interpreter','latex');
title(pTitle1) 
grid on

subplot(323);
hold all;
plot([0:N1]*h1, X1(3,:), '-');
plot([0:N1]*h1, X1(4,:), '-');
ylabel('Angular Velocity (rad/s)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Interpreter','latex');
grid on;

subplot(325);
hold all;
stairs([0:N1-1]*h1, U1, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 inf -3 3]);
grid on;

N2= length(U2);
subplot(322)
hold all;
plot([0:N2]*h2, X2(1,:), '-');
plot([0:N2]*h2, X2(2,:), '-');
ylabel('Angular Position (rad)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\theta_1$','$\theta_2$', 'Interpreter','latex');
title(pTitle2);
grid on;

subplot(324)
hold all;
plot([0:N2]*h2, X2(3,:), '-');
plot([0:N2]*h2, X2(4,:), '-');
ylabel('Angular Velocity (rad/s)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Interpreter','latex');
grid on;

subplot(326)
hold all;
stairs([0:N2-1]*h2, U2, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)');
legend('$u$', 'Interpreter','latex', 'FontSize', 10);
axis([0 inf -3 3]);
grid on;
end


