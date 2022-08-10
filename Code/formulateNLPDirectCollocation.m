function [solver, w0, lbw, ubw, lbg, ubg] = formulateNLPDirectCollocation(d, Q, R, u_max, N, T, modelParams)
% Formulate NLP for direct collocation for DAE.
% This is based on CasADi example "direct_collocation.m" and 
% on chapter 14.4.3, Manuscript of Numerical Optimal Control Course SS2022.
% This function supports cost function L = (x-setPoint)^T*Q*(x-setPoint) + u^T*R*u
% with constraint -u_max < u < m_max. If Q == zero(nx,nx), terminal
% constraint would be activated
% input:
%           d:   Degree of interpolating polynomial
%           Q:   Weight matrix for cost of states
%           R:   Weight matrix for cost of control input
%       u_max:   Upper bound for control input
%           N:   Number of control intervals per horizon
%           T:   Time horizon
% modelParams:   Model parameters
% output:
%      solver:   Solver which takes initial state and setpoint as parameters
%          w0:   Initial guess for decision variables, all zeros
%         lbw:   Lower bound vector for decision variables
%         ubw:   Upper bound vector for decision variables
%         lbg:   Lower bound for constraints
%         ubg:   Upper bound for constraints

import casadi.*

%% Set up necessary terms
% Dimension of variables
nx = 4; % Dimension of state variables
nu = 1; % Dimension of control input
nz = 2; % Dimension of algebraic variables

h = T/N; % Timestep

% State variable [theta1, theta2, theta1_dot, theta2_dot]
x = MX.sym('x', nx, 1);
% Control input
u = MX.sym('u', nu, 1);
% Algebraic variable
z = MX.sym('z', nz, 1);

% Model equations
[ode, alg] = modelDAE(x, z, u, modelParams);

% Objective term 
x_setpoint = MX.sym('x_setpoint', nx, 1);
L = (x-x_setpoint)'*Q*(x-x_setpoint) + u'*R*u;

% Functions used in collocation
f = Function('f', {x, z, u}, {ode, L});
alg = Function('alg', {x, z, u}, {alg});

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


% Build polynomial for interpolation
[B, C, D] = buildPolynomials(d);

% Direct collocation
% Formulate the NLP
x0_hat = MX.sym('x0_hat', nx, 1);

X0 = MX.sym('X0', nx, 1);
w = {w{:}, X0};
% Constraints for x0
lbw = [lbw; -inf; -inf; -inf; -inf];
ubw = [ubw;  inf;  inf; inf; inf];
% Initial guess
w0 = [w0; 0; 0; 0; 0];
% Equality constraints
g = {g{:}, X0 - x0_hat};
lbg = [lbg; 0; 0; 0; 0];
ubg = [ubg; 0; 0; 0; 0];

Xk = x0_hat;

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, Uk};
    lbw = [lbw; -u_max];
    ubw = [ubw; u_max];
    w0 = [w0; 0];

    % State at collocation points
    Xkj = {};
    % Algebraic states at collocation points
    Zkj = {};
    for j=1:d
        Xkj{j} = MX.sym(['X_' num2str(k) '_' num2str(j)], nx, 1);
        w = {w{:}, Xkj{j}};
        lbw = [lbw; -inf; -inf; -inf; -inf];
        ubw = [ubw;  inf;  inf;  inf;  inf];
        w0 = [w0; 0; 0; 0; 0];

        Zkj{j} = MX.sym(['Z_' num2str(k) '_' num2str(j)], nz, 1);
        w = {w{:}, Zkj{j}};
        lbw = [lbw; -inf; -inf];
        ubw = [ubw;  inf;  inf];
        w0 = [w0; 0; 0];
    end

    % Loop over collocation points
    Xk_end = D(1)*Xk;
    for j=1:d
       % Expression for the state derivative at the collocation point
       xp = C(1,j+1)*Xk;
       for r=1:d
           xp = xp + C(r+1,j+1)*Xkj{r};
       end

       % Append collocation equations
       [fj, qj] = f(Xkj{j},Zkj{j}, Uk);
       g = {g{:}, h*fj - xp};
       lbg = [lbg; 0; 0; 0; 0];
       ubg = [ubg; 0; 0; 0; 0];

       g = {g{:}, alg(Xkj{j}, Zkj{j}, Uk)};
       lbg = [lbg; 0; 0];
       ubg = [ubg; 0; 0];

       % Add contribution to the end state
       Xk_end = Xk_end + D(j+1)*Xkj{j};

       % Add contribution to quadrature function
       J = J + B(j+1)*qj*h;
    end

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf; -inf; -inf];
    ubw = [ubw;  inf;  inf; inf; inf];
    w0 = [w0; 0; 0; 0; 0];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0];
end
% Terminal constraint
if (Q == zeros(nx, nx))
    g = [g, {x_setpoint-Xk}];
    lbg = [lbg; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0];
end

% Create an NLP solver
w = vertcat(w{:});
g = vertcat(g{:});
prob = struct('f', J, 'x', w, 'g', g, 'p', [x0_hat; x_setpoint]);
nlpsol_opts = struct;
nlpsol_opts.ipopt.print_level = 0;    % No printing of evaluations
nlpsol_opts.print_time = 0;          % No printing of time
solver = nlpsol('solver', 'ipopt', prob, nlpsol_opts);
end

function [B, C, D] = buildPolynomials(d)
% Build up coefficients for interpolation
% input:
%   d:   Degree of interpolating polynomial
% output:
%   B:  Coefficients of the quadrature function
%   C:  Coefficients of the collocation equation
%   D:  Coefficients of the continuity equation

import casadi.*

% Get collocation points
tau_root = [0 collocation_points(d, 'legendre')];

% Coefficients of the collocation equation
C = zeros(d+1,d+1);

% Coefficients of the continuity equation
D = zeros(d+1, 1);

% Coefficients of the quadrature function
B = zeros(d+1, 1);

% Construct polynomial basis
for j=1:d+1
  % Construct Lagrange polynomials to get the polynomial basis at the collocation point
  coeff = 1;
  for r=1:d+1
    if r ~= j
      coeff = conv(coeff, [1, -tau_root(r)]);
      coeff = coeff / (tau_root(j)-tau_root(r));
    end
  end
  % Evaluate the polynomial at the final time to get the coefficients of the continuity equation
  D(j) = polyval(coeff, 1.0);

  % Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
  pder = polyder(coeff);
  for r=1:d+1
    C(j,r) = polyval(pder, tau_root(r));
  end

  % Evaluate the integral of the polynomial to get the coefficients of the quadrature function
  pint = polyint(coeff);
  B(j) = polyval(pint, 1.0);
end
end

