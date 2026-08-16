function [t, Y] = ode45_solver(dynamics, params)

% ode45 Solver
%
% Solves the drone dynamics using MATLAB's built-in ode45 solver.
%
% NOTE:
% This is a function file.
% Do NOT run this file directly.
% Run main.m instead.


% Time vector
tspan = params.t0:params.h:params.tf;

% Solve the ODE
[t, Y] = ode45(@(t,Y) dynamics(t,Y,params), tspan, params.Y0);

% Transpose so the format matches the other solvers
Y = Y';

% Convert time to a row vector
t = t';

end