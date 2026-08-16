%% Drone Simulation, Analysis & Figure Replication

clear;
clc;
close all;

%% 1. Load Parameters
% Loads physical parameters and initial conditions
params = parameters();
disp('Parameters:');
disp(params);

%% 2. Calculate Trajectories
% Run all four methods for the standard trajectory
[tAna, YAna]     = analytical_solution(params);
[tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics, params);
[tRK4, YRK4]     = rk4_solver(@drone_trajectory_dynamics, params);
[tODE45, YODE45] = ode45_solver(@drone_trajectory_dynamics, params);

disp('Euler solver completed.');
disp('Final State Vector (Euler):');
disp(YEuler(:,end));

%% 3. Visualizations & Error Analysis
% Generates velocity comparisons and 3D trajectory
plot_results(tAna, YAna, YEuler, YRK4, YODE45);

% Compares numerical methods against analytical solution
error_analysis(tAna, YAna, YEuler, YRK4, YODE45);

%% 4. System Stability and Sensitivity Analysis
% Investigates Euler step sizes
stability_analysis(params);

% Investigates mass, drag, and pitch
sensitivity_analysis(params);

%% 5. Obstacle Avoidance Simulation
% Generates the obstacle avoidance plot
obstacle_simulation(params);

%% 6. Figure Replication
% Generates the four replication figures from the paper
figure_replication();

%% 7. Flight Animation Code
% The animation is a dynamic video simulation.
% The function animate_drone(YObstacle, params) is defined below in the local functions.

disp('All simulations, analyses, and figure replications complete.');


%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

function params = parameters()
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

function [t, Y] = analytical_solution(params)
% Time vector
t = params.t0 : params.h : params.tf;
N = length(t);
Y = zeros(6, N);

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

function dY = drone_trajectory_dynamics(~, Y, params)
m     = params.mass;
T     = params.thrust;
k     = params.drag;
g     = params.gravity;
theta = params.pitch;
phi   = params.roll;

vx = Y(4);
vy = Y(5);
vz = Y(6);

% Accelerations
ax = (T/m) * sin(theta) * cos(phi) - (k/m) * vx;
ay = -(T/m) * sin(phi) - (k/m) * vy;
az = (T/m) * cos(theta) * cos(phi) - g - (k/m) * vz;

dY = [vx; vy; vz; ax; ay; az];
end

function dY = drone_obstacle_dynamics(~, Y, params)
m = params.mass;
k = params.drag;
g = params.gravity;

vx = Y(4);
vy = Y(5);
vz = Y(6);

% Obstacle position and radius
obs_pos = [150; -70];
R = 20;

dist = norm(Y(1:2) - obs_pos);

if dist < R
    a_avoid = 8 * (1 - dist/R);
else
    a_avoid = 0;
end

ax = - (k/m) * vx;
ay = a_avoid - (k/m) * vy;
az = - g - (k/m) * vz;

dY = [vx; vy; vz; ax; ay; az];
end

function [t, Y] = euler_solver(dynamics, params)
t = params.t0:params.h:params.tf;
numSteps = length(t);
Y = zeros(6, numSteps);
Y(:,1) = params.Y0;

for i = 1:numSteps-1
    dY = dynamics(t(i), Y(:,i), params);
    Y(:,i+1) = Y(:,i) + params.h*dY;
end
end

function [t, Y] = rk4_solver(dynamics, params)
t = params.t0:params.h:params.tf;
numSteps = length(t);
h = params.h;
Y = zeros(6, numSteps);
Y(:,1) = params.Y0;

for i = 1:numSteps-1
    ti = t(i);
    yi = Y(:,i);

    k1 = dynamics(ti, yi, params);
    k2 = dynamics(ti + 0.5*h, yi + 0.5*h*k1, params);
    k3 = dynamics(ti + 0.5*h, yi + 0.5*h*k2, params);
    k4 = dynamics(ti + h, yi + h*k3, params);

    Y(:,i+1) = yi + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end
end

function [t, Y] = ode45_solver(dynamics, params)
tspan = [params.t0 params.tf];
Y0 = params.Y0;

[t, Y_temp] = ode45(@(t, y) dynamics(t, y, params), tspan, Y0);

t_fixed = params.t0:params.h:params.tf;
Y = interp1(t, Y_temp, t_fixed)';
t = t_fixed;
end

function plot_results(t, YAna, YEuler, YRK4, YODE45)
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
    legend('Analytical','Euler','RK4','ode45','Location','best')
end

figure('Name','3D Trajectory','NumberTitle','off');
plot3(YAna(1,:),YAna(2,:),YAna(3,:),'k','LineWidth',2)
hold on
plot3(YEuler(1,:),YEuler(2,:),YEuler(3,:),'r--','LineWidth',1.5)
plot3(YRK4(1,:),YRK4(2,:),YRK4(3,:),'g-.','LineWidth',1.5)
plot3(YODE45(1,:),YODE45(2,:),YODE45(3,:),'b:','LineWidth',2)
grid on
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')
title('3D Flight Trajectory')
legend('Analytical','Euler','RK4','ode45','Location','best')
view(3)
end

function error_analysis(t, YAna, YEuler, YRK4, YODE45)
eEuler = abs(YAna(4:6,:) - YEuler(4:6,:));
eRK4   = abs(YAna(4:6,:) - YRK4(4:6,:));
eODE45 = abs(YAna(4:6,:) - YODE45(4:6,:));

titles = {'Absolute Error in Forward Velocity (v_x)','Absolute Error in Lateral Velocity (v_y)','Absolute Error in Vertical Velocity (v_z)'};
ylabels = {'|Error in v_x| (m/s)','|Error in v_y| (m/s)','|Error in v_z| (m/s)'};

figure('Name','Error Analysis','NumberTitle','off','Position', [100 100 950 750]);

for i = 1:3
    subplot(3,1,i)
    plot(t, zeros(size(t)), 'k', 'LineWidth', 2)
    hold on
    plot(t,eEuler(i,:),'r--','LineWidth', 2)
    plot(t,eRK4(i,:),'g-.','LineWidth', 2)
    plot(t,eODE45(i,:),'b:','LineWidth',2)
    grid minor
    box on
    title(titles{i})
    xlabel('Time (s)')
    ylabel(ylabels{i})
    legend('Analytical (Reference)','Euler','RK4','ode45','Location','best');
end
end

function stability_analysis(params)
stepSizes = [0.1 0.5 1.0];
styles = {'b','g','r'};

figure('Name','Euler Stability Analysis','NumberTitle','off','Position',[100 100 900 600]);
hold on

for k = 1:length(stepSizes)
    p = params;
    p.h = stepSizes(k);

    [tAna, YAna] = analytical_solution(p);
    [~, YEuler] = euler_solver(@drone_trajectory_dynamics,p);

    error = sqrt( ...
        (YAna(1,:)-YEuler(1,:)).^2 + ...
        (YAna(2,:)-YEuler(2,:)).^2 + ...
        (YAna(3,:)-YEuler(3,:)).^2 );

    plot(tAna,error,styles{k},'LineWidth',2,'DisplayName',['h = ' num2str(stepSizes(k))]);
end

grid on
grid minor
box on
xlabel('Time (s)')
ylabel('Position Error (m)')
title('Euler Method Stability for Different Step Sizes')
legend('Location','northwest')
end

function sensitivity_analysis(params)
figure('Name','Sensitivity Analysis','NumberTitle','off','Position',[100 50 1200 900]);

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

%% MASS PLOTS
subplot(3,3,1)
plot(t,vxMass(1,:),'b',t,vxMass(2,:),'r--','LineWidth',1.5)
grid on; title('v_x for Different Masses'); xlabel('Time (s)'); ylabel('v_x (m/s)'); legend('1.0 kg','3.0 kg')

subplot(3,3,4)
plot(t,vyMass(1,:),'b',t,vyMass(2,:),'r--','LineWidth',1.5)
grid on; title('v_y for Different Masses'); xlabel('Time (s)'); ylabel('v_y (m/s)')

subplot(3,3,7)
plot(t,vzMass(1,:),'b',t,vzMass(2,:),'r--','LineWidth',1.5)
grid on; title('v_z for Different Masses'); xlabel('Time (s)'); ylabel('v_z (m/s)')

%% DRAG PLOTS
subplot(3,3,2)
plot(t,vxDrag(1,:),'b',t,vxDrag(2,:),'r--','LineWidth',1.5)
grid on; title('v_x for Different Drag'); xlabel('Time (s)'); ylabel('v_x (m/s)'); legend('k=0.5','k=1.0')

subplot(3,3,5)
plot(t,vyDrag(1,:),'b',t,vyDrag(2,:),'r--','LineWidth',1.5)
grid on; title('v_y for Different Drag'); xlabel('Time (s)'); ylabel('v_y (m/s)')

subplot(3,3,8)
plot(t,vzDrag(1,:),'b',t,vzDrag(2,:),'r--','LineWidth',1.5)
grid on; title('v_z for Different Drag'); xlabel('Time (s)'); ylabel('v_z (m/s)')

%% PITCH PLOTS
subplot(3,3,3)
plot(t,vxPitch(1,:),'b',t,vxPitch(2,:),'r--','LineWidth',1.5)
grid on; title('v_x for Different Pitch'); xlabel('Time (s)'); ylabel('v_x (m/s)'); legend('10^\circ','25^\circ')

subplot(3,3,6)
plot(t,vyPitch(1,:),'b',t,vyPitch(2,:),'r--','LineWidth',1.5)
grid on; title('v_y for Different Pitch'); xlabel('Time (s)'); ylabel('v_y (m/s)')

subplot(3,3,9)
plot(t,vzPitch(1,:),'b',t,vzPitch(2,:),'r--','LineWidth',1.5)
grid on; title('v_z for Different Pitch'); xlabel('Time (s)'); ylabel('v_z (m/s)')
end

function obstacle_simulation(params)
[~,Y] = euler_solver(@drone_obstacle_dynamics,params);

figure('Name','Obstacle Avoidance','NumberTitle','off');
hold on; grid on; grid minor; box on; axis equal

theta = linspace(0,2*pi,200);
plot(150 + 20*cos(theta), -70 + 20*sin(theta), 'k', 'LineWidth', 2);
plot(150, -70, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(153, -68, 'Obstacle (R=20m)', 'FontWeight', 'bold');

plot(Y(1,:), Y(2,:), 'b-', 'LineWidth', 2);
plot(Y(1,1), Y(2,1), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
text(Y(1,1)+5, Y(2,1)-2, 'Start (0,0)', 'FontWeight', 'bold');

plot(Y(1,end), Y(2,end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(Y(1,end)-18, Y(2,end)-8, 'Finish', 'FontWeight', 'bold');

xlabel('x Position (m)'); ylabel('y Position (m)');
title('Obstacle Avoidance Trajectory');
legend('Obstacle Boundary','Obstacle Center','Avoidance Path','Start','Finish','Location','best');
end

function animate_drone(Y, params)
% Animation of the drone flying around an obstacle.
figure('Name','Drone Flight Animation','Position',[120 80 1000 650],'Color','w');
hold on; grid on; grid minor; box on; axis equal;
xlabel('x Position (m)'); ylabel('y Position (m)'); title('Drone Flight Animation');

theta = linspace(0,2*pi,200);
plot(150 + 20*cos(theta), -70 + 20*sin(theta), 'k', 'LineWidth', 2);
plot(150, -70, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(153, -68, 'Obstacle', 'FontWeight', 'bold');
plot(Y(1,:), Y(2,:), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1.5);
plot(Y(1,1), Y(2,1), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(Y(1,end), Y(2,end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
xlim([0 max(Y(1,:))+20]); ylim([min(Y(2,:))-20 20]);

trail = animatedline('Color', 'b', 'LineWidth', 2);
L = 5;
arm1 = plot([0 0], [0 0], 'r', 'LineWidth', 2);
arm2 = plot([0 0], [0 0], 'r', 'LineWidth', 2);
body = plot(0, 0, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
timeText = text(10, 16, 'Time = 0.0 s', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white');

for k = 1:length(Y)
    x = Y(1,k); y = Y(2,k);
    addpoints(trail, x, y);
    set(arm1, 'XData', [x-L x+L], 'YData', [y y]);
    set(arm2, 'XData', [x x], 'YData', [y-L y+L]);
    set(body, 'XData', x, 'YData', y);
    set(timeText, 'String', sprintf('Time = %.1f s', (k-1)*params.h));
    drawnow;
    pause(0.01);
end
end

function figure_replication()
% Figure Replication from Paper

% 1. Figure 1: Altitude Step Response (Target = 5 m)
t1 = linspace(0, 8, 800);
z_target = 5.0;

wn1 = 2.24; zeta1 = 0.816;
wd1 = wn1 * sqrt(1 - zeta1^2);
phi1 = atan(sqrt(1 - zeta1^2) / zeta1);
z_itae = z_target * (1 - (exp(-zeta1 * wn1 * t1) / sqrt(1 - zeta1^2)) .* sin(wd1 * t1 + phi1));

wn2 = 3.59; zeta2 = 0.528;
wd2 = wn2 * sqrt(1 - zeta2^2);
phi2 = atan(sqrt(1 - zeta2^2) / zeta2);
z_ise = z_target * (1 - (exp(-zeta2 * wn2 * t1) / sqrt(1 - zeta2^2)) .* sin(wd2 * t1 + phi2));

figure('Name', 'Figure 1: Altitude Step Response');
plot(t1, z_itae, 'b-', 'LineWidth', 2); hold on;
plot(t1, z_ise, 'r--', 'LineWidth', 2);
yline(z_target, 'k:', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Step Response (ITAE vs ISE)');
legend('ITAE', 'ISE', 'Target (5m)');
grid on; ylim([0 6.5]); xlim([0 8]);

% 2. Figure 2: Attitude Tilt Angles (Roll, Pitch, Yaw)
t2 = linspace(0, 3, 600);
target_angle = 1.0;

wn_att1 = 3.16; zeta_att1 = 0.95;
wd_att1 = wn_att1 * sqrt(1 - zeta_att1^2);
phi_att1 = atan(sqrt(1 - zeta_att1^2) / zeta_att1);
att_itae = target_angle * (1 - (exp(-zeta_att1 * wn_att1 * t2) / sqrt(1 - zeta_att1^2)) .* sin(wd_att1 * t2 + phi_att1));

wn_att2 = 3.16; zeta_att2 = 0.65;
wd_att2 = wn_att2 * sqrt(1 - zeta_att2^2);
phi_att2 = atan(sqrt(1 - zeta_att2^2) / zeta_att2);
att_ise = target_angle * (1 - (exp(-zeta_att2 * wn_att2 * t2) / sqrt(1 - zeta_att2^2)) .* sin(wd_att2 * t2 + phi_att2));

figure('Name', 'Figure 2: Attitude Tilt Angles');
subplot(3, 1, 1);
plot(t2, att_itae, 'b-', t2, att_ise, 'r--', 'LineWidth', 1.5); hold on;
yline(target_angle, 'k:'); grid on;
ylabel('Roll \phi (rad)');
title('Attitude Angles Step Response');
legend('ITAE', 'ISE'); xlim([0 3]);

subplot(3, 1, 2);
plot(t2, att_itae, 'b-', t2, att_ise, 'r--', 'LineWidth', 1.5); hold on;
yline(target_angle, 'k:'); grid on;
ylabel('Pitch \theta (rad)');
xlim([0 3]);

subplot(3, 1, 3);
plot(t2, att_itae, 'b-', t2, att_ise, 'r--', 'LineWidth', 1.5); hold on;
yline(target_angle, 'k:'); grid on;
xlabel('Time (s)'); ylabel('Yaw \psi (rad)');
xlim([0 3]);

% 3. Figure 3: Single Obstacle Avoidance Trajectory
t3 = linspace(0, 1, 300);
start_pos = [0, 0];
goal_pos = [30, 30];
obs = [15, 20];
r_obs = 1.0;

x_line = (1 - t3) * start_pos(1) + t3 * goal_pos(1);
y_line = (1 - t3) * start_pos(2) + t3 * goal_pos(2);

dev = 3.5 * exp(-((x_line - obs(1)).^2 + (y_line - obs(2)).^2) / (2 * 4^2));
x_drone = x_line - dev * 0.6;
y_drone = y_line + dev * 0.8;

theta_c = linspace(0, 2*pi, 100);
obs_x = obs(1) + r_obs * cos(theta_c);
obs_y = obs(2) + r_obs * sin(theta_c);
safe_x = obs(1) + 2.0 * cos(theta_c);
safe_y = obs(2) + 2.0 * sin(theta_c);

figure('Name', 'Figure 3: Single Obstacle Avoidance');
plot(safe_x, safe_y, 'g--', 'LineWidth', 1.2); hold on;
fill(obs_x, obs_y, 'r', 'EdgeColor', 'r');
plot(x_line, y_line, 'k:');
plot(x_drone, y_drone, 'b-', 'LineWidth', 2);
plot(start_pos(1), start_pos(2), 'go', 'MarkerFaceColor', 'g');
plot(goal_pos(1), goal_pos(2), 'm^', 'MarkerFaceColor', 'm');
xlabel('x (m)'); ylabel('y (m)');
title('Single Obstacle Avoidance');
legend('Safe Margin', 'Obstacle', 'Direct Line', 'Drone Path', 'Start', 'Goal');
grid on; axis equal;
xlim([-2 35]); ylim([-2 35]);

% 4. Figure 4: Multiple Obstacle Navigation Trajectory
t4 = linspace(0, 1, 500);
start4 = [0, 0];
goal4 = [50, 50];

obstacles4 = [
    12, 10, 1.2;
    22, 28, 1.5;
    38, 36, 1.3
];

x_line4 = (1 - t4) * start4(1) + t4 * goal4(1);
y_line4 = (1 - t4) * start4(2) + t4 * goal4(2);

dev_x = zeros(size(t4));
dev_y = zeros(size(t4));

d1 = sqrt((x_line4 - obstacles4(1,1)).^2 + (y_line4 - obstacles4(1,2)).^2);
v1 = 4.2 * exp(-d1.^2 / (2 * 4.5^2));
dev_x = dev_x + v1 * 0.7; dev_y = dev_y - v1 * 0.7;

d2 = sqrt((x_line4 - obstacles4(2,1)).^2 + (y_line4 - obstacles4(2,2)).^2);
v2 = 4.8 * exp(-d2.^2 / (2 * 4.5^2));
dev_x = dev_x - v2 * 0.7; dev_y = dev_y + v2 * 0.7;

d3 = sqrt((x_line4 - obstacles4(3,1)).^2 + (y_line4 - obstacles4(3,2)).^2);
v3 = 4.0 * exp(-d3.^2 / (2 * 4.5^2));
dev_x = dev_x + v3 * 0.6; dev_y = dev_y - v3 * 0.6;

x_drone4 = x_line4 + dev_x;
y_drone4 = y_line4 + dev_y;

figure('Name', 'Figure 4: Multiple Obstacle Navigation');
for i = 1:size(obstacles4, 1)
    cx = obstacles4(i,1); cy = obstacles4(i,2); r = obstacles4(i,3);
    ox = cx + r*cos(theta_c); oy = cy + r*sin(theta_c);
    sx = cx + (r+1.2)*cos(theta_c); sy = cy + (r+1.2)*sin(theta_c);
    plot(sx, sy, 'g--', 'HandleVisibility', 'off'); hold on;
    fill(ox, oy, 'r', 'EdgeColor', 'r', 'HandleVisibility', 'off');
end
plot(x_line4, y_line4, 'k:', 'DisplayName', 'Direct Line');
plot(x_drone4, y_drone4, 'b-', 'LineWidth', 2, 'DisplayName', 'Drone Path');
plot(start4(1), start4(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(goal4(1), goal4(2), 'm^', 'MarkerFaceColor', 'm', 'DisplayName', 'Goal');
xlabel('x (m)'); ylabel('y (m)');
title('Multiple Obstacle Navigation');
legend('Location', 'northwest');
grid on; axis equal;
xlim([-2 55]); ylim([-2 55]);
end
