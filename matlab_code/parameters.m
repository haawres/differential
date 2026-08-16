function params = parameters()

% Parameter used in the simulation

% Drone mass (kg)
params.mass = 1.5; 

% Total thrust (N)
params.thrust = 20;    

% Linear drag coefficient (N.s/m)
params.drag = 0.5;

% Gravity (m/s^2)
params.gravity = 9.81;          


% Drone orientation
% Pitch angle (rad)
params.pitch = deg2rad(10);  

% Roll angle (rad)
params.roll  = deg2rad(5);    

% Simulation parameters
% Start time (s)
params.t0 = 0;       

% End time (s)
params.tf = 45;     

% Default step size
params.h  = 0.1;                

% Initial conditions
% [x y z vx vy vz]

params.Y0 = [0;0;0;0;0;0];

end