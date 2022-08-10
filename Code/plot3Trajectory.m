function [] = plot3Trajectory(X1, U1, h1, pTitle1, X2, U2, h2, pTitle2, X3, U3, h3, pTitle3, u_max)
% Plot 3 trajectories of the pendulum side by side
% input:
%       X1:  Horizontally stacked vectors of state of trajectory 1
%       U1:  Horizontally stacked vectors of control input of trajectory 1
%       h1:  Time step for trajectory 1
%  pTitle1:  Title for the plot of trajectory 1
%       X2:  Horizontally stacked vectors of state of trajectory 2
%       U2:  Horizontally stacked vectors of control input of trajectory 2
%       h2:  Time step for trajectory 2
%  pTitle2:  Title for the plot of trajectory 2
%       X3:  Horizontally stacked vectors of state of trajectory 3
%       U3:  Horizontally stacked vectors of control input of trajectory 3]
%       h3:  Time step for trajectory 3
%  pTitle3:  Title for the plot of trajectory 3
%    u_max:   Upper bound for control input

N1 = length(U1);
subplot(431);
hold all;
plot([0:N1]*h1, X1(1,:), '-');
plot([0:N1]*h1, X1(2,:), '-');
ylabel('Angular Position (rad)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\theta_1$','$\theta_2$', 'Interpreter','latex');
title(pTitle1) 
grid on

subplot(434);
hold all;
plot([0:N1]*h1, X1(3,:), '-');
plot([0:N1]*h1, X1(4,:), '-');
ylabel('Angular Velocity (rad/s)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Interpreter','latex');
grid on;

subplot(437);
hold all;
stairs([0:N1-1]*h1, U1, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 inf -u_max u_max]);
grid on;

subplot(4,3,10);
hold all;
stairs([0:N1-1]*h1, U1, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 2 -u_max u_max]);
grid on;

N2 = length(U2);
subplot(432)
hold all;
plot([0:N2]*h2, X2(1,:), '-');
plot([0:N2]*h2, X2(2,:), '-');
ylabel('Angular Position (rad)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\theta_1$','$\theta_2$', 'Interpreter','latex');
title(pTitle2);
grid on;

subplot(435)
hold all;
plot([0:N2]*h2, X2(3,:), '-');
plot([0:N2]*h2, X2(4,:), '-');
ylabel('Angular Velocity (rad/s)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Interpreter','latex');
grid on;

subplot(438)
hold all;
stairs([0:N2-1]*h2, U2, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 inf -u_max u_max]);
grid on;

subplot(4,3,11);
hold all;
stairs([0:N2-1]*h2, U2, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 2 -u_max u_max]);
grid on;

N3 = length(U3);
subplot(433)
hold all;
plot([0:N3]*h3, X3(1,:), '-');
plot([0:N3]*h3, X3(2,:), '-');
ylabel('Angular Position (rad)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\theta_1$','$\theta_2$', 'Interpreter','latex');
title(pTitle3);
grid on;

subplot(436)
hold all;
plot([0:N3]*h3, X3(3,:), '-');
plot([0:N3]*h3, X3(4,:), '-');
ylabel('Angular Velocity (rad/s)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Interpreter','latex');
grid on;

subplot(439)
hold all;
stairs([0:N3-1]*h3, U3, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 inf -u_max u_max]);
grid on;

subplot(4,3,12);
hold all;
stairs([0:N3-1]*h3, U3, '-');
ylabel('Input Torque (N.m)', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 10);
legend('$u$', 'Interpreter','latex');
axis([0 2 -u_max u_max]);
grid on;

end

