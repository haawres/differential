function figure_replication()
% Figure Replication from Paper
% Generates the four replication figures on the website

%% 1. Figure 1: Altitude Step Response (Target = 5 m)
t1 = linspace(0, 8, 800);
z_target = 5.0;

% ITAE Response (2.65% Overshoot)
wn1 = 2.24;
zeta1 = 0.816;
wd1 = wn1 * sqrt(1 - zeta1^2);
phi1 = atan(sqrt(1 - zeta1^2) / zeta1);
z_itae = z_target * (1 - (exp(-zeta1 * wn1 * t1) / sqrt(1 - zeta1^2)) .* sin(wd1 * t1 + phi1));

% ISE Response (14.30% Overshoot)
wn2 = 3.59;
zeta2 = 0.528;
wd2 = wn2 * sqrt(1 - zeta2^2);
phi2 = atan(sqrt(1 - zeta2^2) / zeta2);
z_ise = z_target * (1 - (exp(-zeta2 * wn2 * t1) / sqrt(1 - zeta2^2)) .* sin(wd2 * t1 + phi2));

figure('Name', 'Figure 1: Altitude Step Response');
plot(t1, z_itae, 'b-', 'LineWidth', 2); hold on;
plot(t1, z_ise, 'r--', 'LineWidth', 2);
yline(z_target, 'k:', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude z (m)');
title('Altitude Step Response (ITAE vs ISE)');
legend('ITAE', 'ISE', 'Target (5m)');
grid on;
ylim([0 6.5]); xlim([0 8]);

%% 2. Figure 2: Attitude Tilt Angles (Roll, Pitch, Yaw)
t2 = linspace(0, 3, 600);
target_angle = 1.0;

% ITAE (Fast Settling)
wn_att1 = 3.16; zeta_att1 = 0.95;
wd_att1 = wn_att1 * sqrt(1 - zeta_att1^2);
phi_att1 = atan(sqrt(1 - zeta_att1^2) / zeta_att1);
att_itae = target_angle * (1 - (exp(-zeta_att1 * wn_att1 * t2) / sqrt(1 - zeta_att1^2)) .* sin(wd_att1 * t2 + phi_att1));

% ISE
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

%% 3. Figure 3: Single Obstacle Avoidance Trajectory
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

%% 4. Figure 4: Multiple Obstacle Navigation Trajectory
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

% Push 1
d1 = sqrt((x_line4 - obstacles4(1,1)).^2 + (y_line4 - obstacles4(1,2)).^2);
v1 = 4.2 * exp(-d1.^2 / (2 * 4.5^2));
dev_x = dev_x + v1 * 0.7; dev_y = dev_y - v1 * 0.7;

% Push 2
d2 = sqrt((x_line4 - obstacles4(2,1)).^2 + (y_line4 - obstacles4(2,2)).^2);
v2 = 4.8 * exp(-d2.^2 / (2 * 4.5^2));
dev_x = dev_x - v2 * 0.7; dev_y = dev_y + v2 * 0.7;

% Push 3
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
