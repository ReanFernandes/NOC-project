function [ode,alg] = modelDAE(x, z, u, params)
% DAE Model for inverted pendulum
% input:
%       x:   state variables
%       z:   algebraic variables
%       u:   control input
%  params:   model parameters
% output:
%     ode:   differentiation of state variables
%     alg:   algebraic equations

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

% ODE
ode = [x(3); x(4); z(1); z(2)];
% ALG
alg = M(x) * z + C(x) * [x(3); x(4)] + G(x) - [u; 0];
end

