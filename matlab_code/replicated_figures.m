%% Quadrotor ISE vs ITAE PID Response (Alanezi et al., 2022 style

clear; clc; close all;

%% Physical parameters (typical small quadrotor)
m  = 1.5;      % mass, kg
l  = 0.25;     % arm length, m
Ix = 0.0232;   % roll inertia, kg*m^2
Iy = 0.0232;   % pitch inertia, kg*m^2
Iz = 0.0117;   % yaw inertia, kg*m^2

K_alt = 1/m;   % plant gain for altitude
K_rp  = l/Ix;  % plant gain for roll & pitch (Ix = Iy)
K_yaw = l/Iz;  % plant gain for yaw

%% PID gains: [Kp Ki Kd], ISE and ITAE tuned
pid_alt_ISE  = [9.16   0.0  3.936];
pid_alt_ITAE = [1.678  0.0  2.399];

pid_rp_ISE   = [17.93  0.0  1.097];
pid_rp_ITAE  = [15.39  0.0  1.611];

pid_yaw_ISE  = [21.63  0.0  1.505];
pid_yaw_ITAE = [8.465  0.0  1.070];

%% Run each channel (RK4, fixed step)
dt = 0.001;

[t_alt, alt_ISE]  = rk4_channel(K_alt, pid_alt_ISE,  5.0, dt, 10);
[~,     alt_ITAE] = rk4_channel(K_alt, pid_alt_ITAE, 5.0, dt, 10);

[t_att, roll_ISE]  = rk4_channel(K_rp, pid_rp_ISE,  0.35, dt, 2);
[~,     roll_ITAE] = rk4_channel(K_rp, pid_rp_ITAE, 0.35, dt, 2);

[~,     pitch_ISE]  = rk4_channel(K_rp, pid_rp_ISE,  0.35, dt, 2);
[~,     pitch_ITAE] = rk4_channel(K_rp, pid_rp_ITAE, 0.35, dt, 2);

[~,     yaw_ISE]  = rk4_channel(K_yaw, pid_yaw_ISE,  0.35, dt, 2);
[~,     yaw_ITAE] = rk4_channel(K_yaw, pid_yaw_ITAE, 0.35, dt, 2);

%% Figure 11 style: Altitude response
figure('Name','Altitude Response');
plot(t_alt, alt_ISE, 'b-.', 'LineWidth', 1.3); hold on;
plot(t_alt, alt_ITAE, 'r--', 'LineWidth', 1.3);
xlabel('Time (seconds)'); ylabel('Altitude (m)');
legend('ISE','ITAE','Location','southeast');
grid on;

%% Figure 12 style: Roll response
figure('Name','Roll Response');
plot(t_att, roll_ISE, 'b-.', 'LineWidth', 1.3); hold on;
plot(t_att, roll_ITAE, 'r--', 'LineWidth', 1.3);
xlabel('Time (seconds)'); ylabel('Roll (rad)');
legend('ISE','ITAE','Location','southeast');
grid on;

%% Figure 13 style: Pitch response
figure('Name','Pitch Response');
plot(t_att, pitch_ISE, 'b-.', 'LineWidth', 1.3); hold on;
plot(t_att, pitch_ITAE, 'r--', 'LineWidth', 1.3);
xlabel('Time (seconds)'); ylabel('Pitch (rad)');
legend('ISE','ITAE','Location','southeast');
grid on;

%% Figure 14 style: Yaw response
figure('Name','Yaw Response');
plot(t_att, yaw_ISE, 'b-.', 'LineWidth', 1.3); hold on;
plot(t_att, yaw_ITAE, 'r--', 'LineWidth', 1.3);
xlabel('Time (seconds)'); ylabel('Yaw (rad)');
legend('ISE','ITAE','Location','southeast');
grid on;

%% ---- local function ----
function [t, y] = rk4_channel(K, pid, target, dt, Tfinal)
% Simulates one PID-controlled double-integrator channel with RK4.
% State x = [y; ydot; ei], ei = running integral of error.
Kp = pid(1); Ki = pid(2); Kd = pid(3);

f = @(x) [ x(2);
           K*(Kp*(target - x(1)) + Ki*x(3) + Kd*(-x(2)));
           target - x(1) ];

n = round(Tfinal/dt);
t = (0:n)'*dt;
x = zeros(3,1);
y = zeros(n+1,1);
y(1) = x(1);

for i = 1:n
    k1 = f(x);
    k2 = f(x + dt/2*k1);
    k3 = f(x + dt/2*k2);
    k4 = f(x + dt*k3);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    y(i+1) = x(1);
end
end
