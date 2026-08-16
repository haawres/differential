%% Compilation of codes
clear;
clc;
close all;

%% 1. Load Parameters
% Loads physical parameters and initial conditions
params = parameters();

%% 2. Calculate Trajectories
% Run all four methods for the standard trajectory
[tAna, YAna]     = analytical_solution(params);
[tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics, params);
[tRK4, YRK4]     = rk4_solver(@drone_trajectory_dynamics, params);
[tODE45, YODE45] = ode45_solver(@drone_trajectory_dynamics, params);

% Display message from original main.m
disp(YEuler(:,end));

%% 3. Visualizations & Error Analysis
% Generates velocity comparisons and 3D trajectory
plot_results(tAna, YAna, YEuler, YRK4, YODE45);

% Compares numerical methods against analytical solution
error_analysis(tAna, YAna, YEuler, YRK4, YODE45);

%% 4. System Analyses
% Investigates Euler step sizes
stability_analysis(params);

% Investigates mass, drag, and pitch
sensitivity_analysis(params);

disp('All simulations and analyses complete!');

%% 6. Replicated Figures
% Quadrotor ISE vs ITAE PID Response (Alanezi et al., 2022 style

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


%% LOCAL FUNCTIONS 

function params = parameters()

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


function [t, Y] = analytical_solution(params)

% Analytical Solution
%
% Calculates the analytical solution for the drone motion.



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


function dYdt = drone_trajectory_dynamics(~, Y, params)
% Calculates the rate of change of the drone states.
%
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



function [t, Y] = euler_solver(dynamics, params)

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


function [t, Y] = rk4_solver(dynamics, params)
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


function [t, Y] = ode45_solver(dynamics, params)

% ode45 Solver
% Solves the drone dynamics using MATLAB's built-in ode45 solver.



% Time vector
tspan = params.t0:params.h:params.tf;

% Solve the ODE
[t, Y] = ode45(@(t,Y) dynamics(t,Y,params), tspan, params.Y0);

% Transpose so the format matches the other solvers
Y = Y';

% Convert time to a row vector
t = t';

end


function plot_results(t, YAna, YEuler, YRK4, YODE45)

% Plot Results
% This function generates all the comparison plots for thenumerical methods.


%% Velocity Comparison

figure('Name','Velocity Comparison','NumberTitle','off');

labels = {'v_x','v_y','v_z'};

for i = 1:3

    subplot(3,1,i)

    plot(t,YAna(i+3,:),'k','LineWidth',2)
    hold on

    plot(t,YEuler(i+3,:),'r--','LineWidth',1.5)

    plot(t,YRK4(i+3,:),'g-.','LineWidth',1.5)

    plot(t,YODE45(i+3,:),'b:','LineWidth',2)

    grid on

    xlabel('Time (s)')
    ylabel(labels{i} + " (m/s)")
    title(labels{i} + " Comparison")

    legend('Analytical','Euler','RK4','ode45',...
        'Location','best')

end


%% 3D Trajectory

figure('Name','3D Trajectory','NumberTitle','off');

plot3(YAna(1,:),YAna(2,:),YAna(3,:),...
    'k','LineWidth',2)

hold on

plot3(YEuler(1,:),YEuler(2,:),YEuler(3,:),...
    'r--','LineWidth',1.5)

plot3(YRK4(1,:),YRK4(2,:),YRK4(3,:),...
    'g-.','LineWidth',1.5)

plot3(YODE45(1,:),YODE45(2,:),YODE45(3,:),...
    'b:','LineWidth',2)

grid on

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

title('Drone Flight Trajectory')

legend('Analytical','Euler','RK4','ode45',...
    'Location','best')

view(3)

end


function error_analysis(t, YAna, YEuler, YRK4, YODE45)

% Error Analysis
% Compares the numerical methods against the analyticalsolution by plotting the absolute error in velocity.


%% Calculate Absolute Errors

eEuler = abs(YAna(4:6,:) - YEuler(4:6,:));
eRK4   = abs(YAna(4:6,:) - YRK4(4:6,:));
eODE45 = abs(YAna(4:6,:) - YODE45(4:6,:));

titles = {'Absolute Error in Forward Velocity (v_x)','Absolute Error in Lateral Velocity (v_y)','Absolute Error in Vertical Velocity (v_z)'};

ylabels = {'|Error in v_x| (m/s)','|Error in v_y| (m/s)','|Error in v_z| (m/s)'};

figure( ...
    'Name','Error Analysis', ...
    'NumberTitle','off', ...
    'Position', [100 100 950 750]);

for i = 1:3

    subplot(3,1,i)

    plot(t, zeros(size(t)), 'k', 'LineWidth', 2)
    hold on

    plot(t,eEuler(i,:),'r--','LineWidth', 2)
    hold on

    plot(t,eRK4(i,:),'g-.','LineWidth', 2)

    plot(t,eODE45(i,:),'b:','LineWidth',2)

    grid minor
    box on

    title(titles{i})
    xlabel('Time (s)')
    ylabel(ylabels{i})

    legend('Analytical (Reference)',...
        'Euler', ...
        'RK4', ...
        'ode45', ...
        'Location','best');

end

end


function stability_analysis(params)
% Stability Analysis
% This function investigates how the Euler method behaveswhen different step sizes are used.

% Step sizes to test
stepSizes = [0.1 0.5 1.0];

% Line styles
styles = {'b','g','r'};

figure('Name','Euler Stability Analysis',...
    'NumberTitle','off',...
    'Position',[100 100 900 600]);

hold on

for k = 1:length(stepSizes)

    % Copy parameters
    p = params;

    % Change step size
    p.h = stepSizes(k);

    % Analytical solution
    [tAna, YAna] = analytical_solution(p);

    % Euler solution
    [tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics,p);

    % Position error
    error = sqrt( ...
        (YAna(1,:)-YEuler(1,:)).^2 + ...
        (YAna(2,:)-YEuler(2,:)).^2 + ...
        (YAna(3,:)-YEuler(3,:)).^2 );

    plot(tAna,error,...
        styles{k},...
        'LineWidth',2,...
        'DisplayName',['h = ' num2str(stepSizes(k))]);

end

grid on
grid minor
box on

xlabel('Time (s)','FontSize',12)
ylabel('Position Error (m)','FontSize',12)

title('Euler Stability for Different Step Sizes','FontSize',14)

legend('Location','northwest')

end


function sensitivity_analysis(params)
% Sensitivity Analysis
% Investigates how mass, drag and pitch affect the drone velocities.

figure('Name','Sensitivity Analysis',...
       'NumberTitle','off',...
       'Position',[100 50 1200 900]);

%% MASS 
massValues = [1.0 3.0];

for j = 1:length(massValues)

    p = params;
    p.mass = massValues(j);

    [t,Y] = analytical_solution(p);

    vxMass(j,:) = Y(4,:);
    vyMass(j,:) = Y(5,:);
    vzMass(j,:) = Y(6,:);

end

%% DRAG 
dragValues = [0.5 1.0];

for j = 1:length(dragValues)

    p = params;
    p.drag = dragValues(j);

    [t,Y] = analytical_solution(p);

    vxDrag(j,:) = Y(4,:);
    vyDrag(j,:) = Y(5,:);
    vzDrag(j,:) = Y(6,:);

end

%% PITCH
pitchValues = [10 25];

for j = 1:length(pitchValues)

    p = params;
    p.pitch = deg2rad(pitchValues(j));

    [t,Y] = analytical_solution(p);

    vxPitch(j,:) = Y(4,:);
    vyPitch(j,:) = Y(5,:);
    vzPitch(j,:) = Y(6,:);

end

%% MASS 

subplot(3,3,1)
plot(t,vxMass(1,:),'k','LineWidth',2)
hold on
plot(t,vxMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('1 kg','3 kg')

subplot(3,3,2)
plot(t,vyMass(1,:),'k','LineWidth',2)
hold on
plot(t,vyMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('1 kg','3 kg')

subplot(3,3,3)
plot(t,vzMass(1,:),'k','LineWidth',2)
hold on
plot(t,vzMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('1 kg','3 kg')

%% DRAG

subplot(3,3,4)
plot(t,vxDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vxDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('k = 0.5','k = 1.0')

subplot(3,3,5)
plot(t,vyDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vyDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('k = 0.5','k = 1.0')

subplot(3,3,6)
plot(t,vzDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vzDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('k = 0.5','k = 1.0')

%% PITCH 

subplot(3,3,7)
plot(t,vxPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vxPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('10°','25°')

subplot(3,3,8)
plot(t,vyPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vyPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('10°','25°')

subplot(3,3,9)
plot(t,vzPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vzPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('10°','25°')

%% All printed results

fprintf('\n PARAMETER STUDY \n');

fprintf('\nMass\n');
fprintf('1 kg Final Speed : %.2f m/s\n',norm([vxMass(1,end),vyMass(1,end),vzMass(1,end)]));
fprintf('3 kg Final Speed : %.2f m/s\n',norm([vxMass(2,end),vyMass(2,end),vzMass(2,end)]));

fprintf('\nDrag\n');
fprintf('k = 0.5 Final Speed : %.2f m/s\n',norm([vxDrag(1,end),vyDrag(1,end),vzDrag(1,end)]));
fprintf('k = 1.0 Final Speed : %.2f m/s\n',norm([vxDrag(2,end),vyDrag(2,end),vzDrag(2,end)]));

fprintf('\nPitch\n');
fprintf('10 deg Final Speed : %.2f m/s\n',norm([vxPitch(1,end),vyPitch(1,end),vzPitch(1,end)]));
fprintf('25 deg Final Speed : %.2f m/s\n',norm([vxPitch(2,end),vyPitch(2,end),vzPitch(2,end)]));

fprintf('\n');

end



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
