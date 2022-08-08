function [xdot] = modelODE(x, u, params)
% ODE Model for inverted pendulum
% input:
%       x:   state variables
%       u:   control input
%  params:   model parameters
% output:
%    xdot:   differentiation of state variables

% Model parameters
m2 = params.m2;
L1 = params.L1;
L2 = params.L2;
I1 = params.I1; 
I2= params.I2;
g = params.g;

% Model
% Euler-Lagrange Matrices
M = @(x) [I1 + m2*L1^2 + 1/4*m2*L2^2*(sin(x(2)))^2,    -1/2*m2*L2*L1*cos(x(2));
          -1/2*m2*L2*L1*cos(x(2)),                    I2 + 1/4*m2*L2^2      ];

C = @(x) [1/2*m2*L2^2*sin(x(2))*cos(x(2))*x(4),     1/2*m2*L2*L1*sin(x(2))*x(4);
          -1/4*m2*L2^2*sin(x(2))*cos(x(2))*x(3),    0                         ];

G = @(x) [0; -1/2*m2*L2*g*sin(x(2))];

% Model equations
xdot = [x(3); x(4); M(x) \ (-C(x) * [x(3); x(4)] - G(x) + [u; 0])];
end

