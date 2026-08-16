function [t, Y] = analytical_solution(params)

% Analytical Solution
%
% Calculates the analytical solution for the drone motion.
%
% NOTE:
% This is a function file.
% Do NOT run this file directly.
% Run main.m instead.


% Time vector
t = params.t0 : params.h : params.tf;

% Number of points
N = length(t);

% Allocate memory
Y = zeros(6, N);

% Read parameters
m = params.mass;
T = params.thrust;
k = params.drag;
g = params.gravity;

theta = params.pitch;
phi = params.roll;

% Constant accelerations
Ax = (T/m) * sin(theta) * cos(phi);
Ay = -(T/m) * sin(phi);
Az = (T/m) * cos(theta) * cos(phi) - g;

% Initial conditions
x0  = params.Y0(1);
y0  = params.Y0(2);
z0  = params.Y0(3);

vx0 = params.Y0(4);
vy0 = params.Y0(5);
vz0 = params.Y0(6);

for i = 1:N

    ti = t(i);

    % Velocity
    vx = (vx0 - Ax*m/k)*exp(-k*ti/m) + Ax*m/k;
    vy = (vy0 - Ay*m/k)*exp(-k*ti/m) + Ay*m/k;
    vz = (vz0 - Az*m/k)*exp(-k*ti/m) + Az*m/k;

    % Position
    x = x0 + (vx0 - Ax*m/k)*(m/k)*(1-exp(-k*ti/m)) + (Ax*m/k)*ti;

    y = y0 + (vy0 - Ay*m/k)*(m/k)*(1-exp(-k*ti/m)) + (Ay*m/k)*ti;

    z = z0 + (vz0 - Az*m/k)*(m/k)*(1-exp(-k*ti/m)) + (Az*m/k)*ti;

    % Store results
    Y(:,i) = [x; y; z; vx; vy; vz];

end

end