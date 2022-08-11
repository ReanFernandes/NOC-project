function [] = animateIP(states, dt, L1, L2, savePath, fileName)
% Animate the double inverted pendulum trajectory, creates a .gif file.
% This is based on "animatePendulum.m" file in exercise 6,
% Numerical Optimal Control Course SS2022
%
% input:
%       states:   Horizontally stacked vectors of position state variables
%           dt:   Timestep between succesive values of theta (in animation)
%           L1:   Length of rotary arm
%           L2:   Length of pendulum
%     savePath:   Location where gif file is saved
%     fileName:   Filename of created gif (string)

if ~isfolder(savePath)
    mkdir(savePath);
end
fileName = fullfile(savePath,fileName);

first_it = true;
% Append initial position vector multiple times, so that swinging up phase 
% could be seen easier in the animation
X_ani = states;
X_ani = [repmat(states(:, 1), 1, 20), X_ani];
fig = figure();
for x = X_ani
    clf; hold on;

    [x1, y1, z1, x2, y2, z2] = getEndpointPosition(x(1), x(2), L1, L2);

    % Limit length for each axis
    axesLimit = 0.5;

    % Plot rotary axis
    plot3( [0, 0], [0, 0], [-axesLimit, axesLimit], '-', "LineWidth", 0.5 , 'color', 'black')

    % Plot rotary arm and pendulum
    plot3( [0, x1], [0, y1], [0, z1], '-', "LineWidth", 2, 'color', '#0072BD')
    plot3( [x1, x2], [y1, y2], [z1, z2], '-', "LineWidth", 2, 'color', '#D95319')

    % Plot 2 endpoints
    plot3(x1, y1, z1, '.', 'MarkerSize', 20)
    plot3(x2, y2, z2, '.', 'MarkerSize', 20)
    
    axis(axesLimit * [-1, 1, -1, 1, -1, 1])
    grid on
    set(gca,'XColor','none','YColor','none', 'ZColor','none','TickDir','out')
    % The view could be adjusted
    view([20 25 5])
    pause(dt)

    frame = getframe(gcf);

    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if first_it
      first_it = false;
      imwrite(imind,cm,fileName,'gif','DelayTime', dt, 'Loopcount',inf);
    else
      imwrite(imind,cm,fileName,'gif','DelayTime', dt, 'WriteMode','append');
    end
end
end

function [x1, y1, z1, x2, y2, z2] = getEndpointPosition(theta1, theta2, L1, L2)
% Compute endpoints of links for visualization
% input:
%     theta1:   Angular position of rotary arm
%     theta2:   Angular position of pendulum
%     L1:       Length of rotary arm
%     L2:       Length of pendulum
% output:
%     x1:       x-position of endpoint of rotary arm
%     y1:       y-position of endpoint of rotary arm
%     x2:       x-position of endpoint of pendulum
%     y2:       y-position of endpoint of pendulum
%     z2:       z-position of endpoint of pendulum

    % Endpoint coordination of rotary arm
    x1 = L1*cos(theta1);
    y1 = L1*sin(theta1);
    z1 = 0;

    % Homogeneous Matrix
    M = [cos(theta1),       -sin(theta1),     0,    L1*cos(theta1);
         sin(theta1),       cos(theta1),      0,    L1*sin(theta1);
         0,                 0,                1,                 0;
         0,                 0,                0,                 1];
    % Coordinate of endpoint of pendulum in reference frame of endpoint of
    % rotary arm
    x21 = 0;
    y21 = L2*sin(theta2);
    z21 = L2*cos(theta2);
    % Endpoint coordination of pendulum
    p2 = M*[x21; y21; z21; 1];
    x2 = p2(1);
    y2 = p2(2);
    z2 = p2(3);
end