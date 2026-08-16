function dY = drone_dynamics(t, Y, params, ctrl_mode)
% =========================================================================
% Standard 6-DOF Quadrotor Dynamics ODE Model
% Mathematical Formulation: Newton-Euler Formalism
% Reference: Alanezi et al., MDPI Drones 2022, 6, 288 (Eqs. 1-12)
% =========================================================================

if nargin < 4
    ctrl_mode = 'itae';
end

% State variables
% Position (m) & Velocity (m/s) in Earth frame
x  = Y(1);  y  = Y(2);  z  = Y(3);
vx = Y(4);  vy = Y(5);  vz = Y(6);

% Euler angles (rad) & Angular rates (rad/s)
phi   = Y(7);   theta = Y(8);   psi = Y(9);   % Roll, Pitch, Yaw
p     = Y(10);  q     = Y(11);  r   = Y(12);  % Body angular rates

% Parameters
m  = params.mass;
g  = params.gravity;
l  = params.arm_len;
Ix = params.Ix;
Iy = params.Iy;
Iz = params.Iz;
Jr = params.Jr;

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

% Altitude Control (Target = params.target_z)
z_err = params.target_z - z;
U1 = m * (g + kp_z * z_err - kd_z * vz) / (cos(phi) * cos(theta) + 1e-6);
U1 = max(0, min(U1, 40)); % Saturation limit

% Attitude Control (Hover stabilization)
phi_des = 0; theta_des = 0; psi_des = 0;
U2 = kp_att * (phi_des - phi) - kd_att * p;     % Roll torque
U3 = kp_att * (theta_des - theta) - kd_att * q; % Pitch torque
U4 = kp_att * (psi_des - psi) - kd_att * r;     % Yaw torque

% Residual rotor speed (hover balance)
omega_r = 0;

%% 1. Translational Kinematics ODEs (Earth Frame)
dx = vx;
dy = vy;
dz = vz;

dvx = (U1 / m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
dvy = (U1 / m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
dvz = (U1 / m) * (cos(phi)*cos(theta)) - g;

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
