function dYdt = drone_trajectory_dynamics(~, Y, params)
% Drone Dynamic Model
% Calculates the rate of change of the drone states.
% State Vector:
% Y = [x y z vx vy vz]


% Extract the velocity components
vx = Y(4);
vy = Y(5);
vz = Y(6);

% Read the physical parameters
m = params.mass;
T = params.thrust;
k = params.drag;
g = params.gravity;

theta = params.pitch;
phi = params.roll;

% Position derivatives
dx = vx;
dy = vy;
dz = vz;

% Velocity derivatives
dvx = (T/m) * sin(theta) * cos(phi) - (k/m) * vx;

dvy = -(T/m) * sin(phi) - (k/m) * vy;

dvz = (T/m) * cos(theta) * cos(phi) - g - (k/m) * vz;

% Return all derivatives
dYdt = [dx;
    dy;
    dz;
    dvx;
    dvy;
    dvz];

end