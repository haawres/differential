function [t, Y] = euler_solver(dynamics, params)
% NOTE:
% This is a function file.
% Do NOT run this file directly.
% Run main.m instead.

% Euler Solver
%
% Solves first-order ODEs using the Euler Method.


% Time vector
t = params.t0:params.h:params.tf;

% Number of time steps
numSteps = length(t);

% Preallocate memory
Y = zeros(6, numSteps);

% Initial condition
Y(:,1) = params.Y0;

% Euler Integration
for i = 1:numSteps-1

    dY = dynamics(t(i), Y(:,i), params);

    Y(:,i+1) = Y(:,i) + params.h*dY;

end

end