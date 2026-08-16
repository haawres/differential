function params = parameters()
% =========================================================================
% MATH221 & CE122: Quadrotor UAV Physical Parameters & Simulation Config
% Compatible with MATLAB R2024b
% Reference: Alanezi et al., MDPI Drones 2022, 6, 288
% =========================================================================

%% Physical & Dynamic Constants (Table 1 from Paper)
params.mass     = 1.888;          % Total quadrotor mass (kg)
params.arm_len  = 0.225;          % Arm length from CG to motor (m)
params.gravity  = 9.81;           % Gravitational acceleration (m/s^2)

% Moments of Inertia (kg*m^2)
params.Ix       = 1.453e-2;       % Roll inertia
params.Iy       = 1.453e-2;       % Pitch inertia
params.Iz       = 2.884e-2;       % Yaw inertia
params.Jr       = 2.820e-7;       % Rotor moment of inertia (kg*m^2)

% Aerodynamic Factors
params.thrust_b = 6.317e-4;       % Thrust coefficient (N*s^2/rad^2)
params.drag_d   = 1.610e-4;       % Drag coefficient (N*m*s^2/rad^2)
params.body_drag = 0.45;          % Translational body drag (N*s/m) - Modification

%% Controller PID Gains (Optimized via Genetic Algorithm)
% ITAE Tuning (Recommended / Best Performance)
params.pid.itae.alt_kp = 5.0000;
params.pid.itae.alt_ki = 2.5728;
params.pid.itae.alt_kd = 3.6485;
params.pid.itae.att_kp = 9.9995;
params.pid.itae.att_ki = 0.000064;
params.pid.itae.att_kd = 1.0915;

% ISE Tuning (Comparison Baseline)
params.pid.ise.alt_kp = 12.8750;
params.pid.ise.alt_ki = 6.6843;
params.pid.ise.alt_kd = 4.5464;
params.pid.ise.att_kp = 9.9995;
params.pid.ise.att_ki = 0.0254;
params.pid.ise.att_kd = 0.7545;

%% Default Simulation Time Settings
params.t0       = 0;              % Initial time (s)
params.tf       = 40;             % Final time (s)
params.dt       = 0.02;           % Fixed time step for Euler / RK4 (s)
params.target_z = 5.0;            % Target flight altitude (m)

%% Environmental Disturbances (Project Modifications)
params.wind_x   = 0.0;            % Lateral wind force (N)
params.wind_y   = 0.0;            % Longitudinal wind force (N)
params.gust_amp = 0.0;            % Gust amplitude (N)
params.gust_freq = 0.5;           % Gust frequency (Hz)

%% Initial State Vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]'
params.Y0 = zeros(12, 1);

end
