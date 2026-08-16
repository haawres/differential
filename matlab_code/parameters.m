function params = parameters()

% AP GROUP 5
% Drone Simulation Project - Submission 3
%
% This file stores all the physical parameters used
% throughout the simulation.

params.mass = 1.5;              % Drone mass (kg)
params.thrust = 20;             % Total thrust (N)
params.drag = 0.5;              % Linear drag coefficient (N.s/m)
params.gravity = 9.81;          % Gravity (m/s^2)

% Drone orientation
params.pitch = deg2rad(10);     % Pitch angle (rad)
params.roll  = deg2rad(5);      % Roll angle (rad)

% Simulation settings
params.t0 = 0;                  % Start time (s)
params.tf = 45;                 % End time (s)
params.h  = 0.1;                % Default step size

% Initial conditions
% Format:
% [x y z vx vy vz]

params.Y0 = [0;0;0;0;0;0];

end