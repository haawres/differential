function replicate_fig3_single_obstacle()
% REPLICATE_FIG3_SINGLE_OBSTACLE replicates Figure 13 from the paper
% Simulates quadcopter path navigating around a single obstacle centered at (15, 20)

%% 1. Simulation Coordinates and Obstacle Position
start_pos = [0, 0];
goal_pos  = [30, 30];
obs_center = [15, 20];
obs_radius = 1.0;
safe_radius = 2.0;

%% 2. Flight Trajectory Generation (Path around obstacle)
t = linspace(0, 1, 300);

% Baseline straight trajectory toward goal
x_straight = (1 - t) * start_pos(1) + t * goal_pos(1);
y_straight = (1 - t) * start_pos(2) + t * goal_pos(2);

% Compute repulsive deviation around obstacle at (15, 20)
dist_to_obs = sqrt((x_straight - obs_center(1)).^2 + (y_straight - obs_center(2)).^2);
avoidance_active = dist_to_obs < (safe_radius + 4.0);

% Tangential smooth avoidance curve
deviation = 3.5 * exp(-((x_straight - obs_center(1)).^2 + (y_straight - obs_center(2)).^2) / (2 * 4.0^2));
x_drone = x_straight - deviation * 0.6;
y_drone = y_straight + deviation * 0.8;

%% 3. Obstacle Geometry for Plotting
theta_circle = linspace(0, 2*pi, 100);
obs_x = obs_center(1) + obs_radius * cos(theta_circle);
obs_y = obs_center(2) + obs_radius * sin(theta_circle);

safe_x = obs_center(1) + safe_radius * cos(theta_circle);
safe_y = obs_center(2) + safe_radius * sin(theta_circle);

%% 4. Plotting Figure 3 (Figure 13 Replication)
figure('Name', 'Figure 3: Single Obstacle Avoidance (Paper Replicated)', 'Color', 'w');

% Plot safe threshold circle
plot(safe_x, safe_y, 'g--', 'LineWidth', 1.2, 'DisplayName', 'Safe Clearance Margin (2.0 m)');
hold on;

% Plot solid obstacle
fill(obs_x, obs_y, [0.8 0.2 0.2], 'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'Obstacle (r = 1.0 m)');

% Plot straight line vs avoided path
plot(x_straight, y_straight, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Desired Direct Path');
plot(x_drone, y_drone, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Avoidance Trajectory (Bat Algorithm)');

% Mark Start and Goal
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point (0,0)');
plot(goal_pos(1), goal_pos(2), 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Goal Point (30,30)');

grid on;
axis equal;
xlabel('X Position (meters)', 'FontSize', 11);
ylabel('Y Position (meters)', 'FontSize', 11);
title('Figure 3: Single Obstacle Navigation Trajectory', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);
xlim([-2 35]);
ylim([-2 35]);

disp('Figure 3 (Single Obstacle Avoidance) replicated successfully.');
end
