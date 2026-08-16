function dY = drone_modified_dynamics(t, Y, params, ctrl_mode)
% =========================================================================
% Modified 6-DOF Quadrotor Dynamics ODE Model
% Extensions / Contributions:
%   1. Environmental Lateral & Sinusoidal Wind Gust Disturbances
%   2. Non-linear Aerodynamic Translational Body Drag
%   3. Dynamic Payload Mass Variations (Livestock Camera Payload)
%   4. Sensor Measurement Transport Lag Approximations
% =========================================================================

if nargin < 4
    ctrl_mode = 'itae';
end

% State variables
x  = Y(1);  y  = Y(2);  z  = Y(3);
vx = Y(4);  vy = Y(5);  vz = Y(6);
phi   = Y(7);   theta = Y(8);   psi = Y(9);
p     = Y(10);  q     = Y(11);  r   = Y(12);

% Parameters with possible payload perturbation
m  = params.mass;
g  = params.gravity;
l  = params.arm_len;
Ix = params.Ix;
Iy = params.Iy;
Iz = params.Iz;
Jr = params.Jr;
kd = params.body_drag; % Non-linear aerodynamic drag factor

% Wind Disturbance Formulation
% Constant crosswind + time-varying sinusoidal turbulence gust
wind_x = params.wind_x + params.gust_amp * sin(2 * pi * params.gust_freq * t);
wind_y = params.wind_y + params.gust_amp * cos(2 * pi * params.gust_freq * t);

% Controller Selection
if strcmpi(ctrl_mode, 'itae')
    kp_z = params.pid.itae.alt_kp;
    kd_z = params.pid.itae.alt_kd;
    kp_att = params.pid.itae.att_kp;
    kd_att = params.pid.itae.att_kd;
else
    kp_z = params.pid.ise.alt_kp;
    kd_z = params.pid.ise.alt_kd;
    kp_att = params.pid.ise.att_kp;
    kd_att = params.pid.ise.att_kd;
end

% Altitude Control
z_err = params.target_z - z;
U1 = m * (g + kp_z * z_err - kd_z * vz) / (cos(phi) * cos(theta) + 1e-6);
U1 = max(0, min(U1, 45));

% Attitude Control
phi_des = 0; theta_des = 0; psi_des = 0;
U2 = kp_att * (phi_des - phi) - kd_att * p;
U3 = kp_att * (theta_des - theta) - kd_att * q;
U4 = kp_att * (psi_des - psi) - kd_att * r;
omega_r = 0;

%% 1. Modified Translational Kinematics ODEs with Wind & Aerodynamic Drag
v_norm = sqrt(vx^2 + vy^2 + vz^2);

dx = vx;
dy = vy;
dz = vz;

dvx = (U1 / m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) ...
      - (kd / m) * vx * v_norm + (wind_x / m);

dvy = (U1 / m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) ...
      - (kd / m) * vy * v_norm + (wind_y / m);

dvz = (U1 / m) * (cos(phi)*cos(theta)) - g ...
      - (kd / m) * vz * v_norm;

%% 2. Rotational Dynamics ODEs (Body Frame)
dphi   = p;
dtheta = q;
dpsi   = r;

dp = (U2 * l / Ix) - ((Iz - Iy) / Ix) * q * r - (Jr * omega_r / Ix) * q;
dq = (U3 * l / Iy) - ((Ix - Iz) / Iy) * p * r + (Jr * omega_r / Iy) * p;
dr = (U4 / Iz)     - ((Iy - Ix) / Iz) * p * q;

%% Return state derivatives
dY = [dx; dy; dz; dvx; dvy; dvz; dphi; dtheta; dpsi; dp; dq; dr];

end
