function [t, Y] = rk4_solver(dynamics, params)

% RK4 Solver
%
% Solves a system of first-order ODEs using the
% Fourth-Order Runge-Kutta Method.
%
% NOTE:
% This is a function file.
% Do NOT run this file directly.
% Run main.m instead.


% Create the time vector
t = params.t0 : params.h : params.tf;

% Number of time steps
numSteps = length(t);

% Number of state variables
numStates = length(params.Y0);

% Preallocate memory
Y = zeros(numStates, numSteps);

% Initial conditions
Y(:,1) = params.Y0;

% RK4 Integration
for i = 1:numSteps-1

    k1 = dynamics(t(i), Y(:,i), params);

    k2 = dynamics(t(i) + params.h/2,Y(:,i) + (params.h/2)*k1,params);

    k3 = dynamics(t(i) + params.h/2,Y(:,i) + (params.h/2)*k2,params);

    k4 = dynamics(t(i) + params.h,Y(:,i) + params.h*k3,params);

    Y(:,i+1) = Y(:,i) + (params.h/6) * (k1 + 2*k2 + 2*k3 + k4);

end

end