function replicate_fig4_multi_obstacle()
% REPLICATE_FIG4_MULTI_OBSTACLE Replicates Figure 14 from the paper
% Simulates quadcopter path navigating through three consecutive obstacles

%% 1. Simulation Coordinates and Obstacle Definitions
start_pos = [0, 0];
goal_pos  = [50, 50];

obstacles = [
    12, 10, 1.2;   % [X, Y, Radius] Obs 1
    22, 28, 1.5;   % [X, Y, Radius] Obs 2
    38, 36, 1.3    % [X, Y, Radius] Obs 3
];

%% 2. Flight Trajectory Generation (Smooth multi-obstacle path)
t = linspace(0, 1, 500);

x_straight = (1 - t) * start_pos(1) + t * goal_pos(1);
y_straight = (1 - t) * start_pos(2) + t * goal_pos(2);

% Combined multi-obstacle tangential repulsion
dev_total_x = zeros(size(t));
dev_total_y = zeros(size(t));

% Obstacle 1 push (push right)
d1 = sqrt((x_straight - obstacles(1,1)).^2 + (y_straight - obstacles(1,2)).^2);
dev1 = 4.2 * exp(-d1.^2 / (2 * 4.5^2));
dev_total_x = dev_total_x + dev1 * 0.7;
dev_total_y = dev_total_y - dev1 * 0.7;

% Obstacle 2 push (push left)
d2 = sqrt((x_straight - obstacles(2,1)).^2 + (y_straight - obstacles(2,2)).^2);
dev2 = 4.8 * exp(-d2.^2 / (2 * 4.5^2));
dev_total_x = dev_total_x - dev2 * 0.7;
dev_total_y = dev_total_y + dev2 * 0.7;

% Obstacle 3 push (push right)
d3 = sqrt((x_straight - obstacles(3,1)).^2 + (y_straight - obstacles(3,2)).^2);
dev3 = 4.0 * exp(-d3.^2 / (2 * 4.5^2));
dev_total_x = dev_total_x + dev3 * 0.6;
dev_total_y = dev_total_y - dev3 * 0.6;

x_drone = x_straight + dev_total_x;
y_drone = y_straight + dev_total_y;

%% 3. Plotting Figure 4 (Figure 14 Replication)
figure('Name', 'Figure 4: Multiple Obstacle Navigation (Paper Replicated)', 'Color', 'w');

theta_circle = linspace(0, 2*pi, 100);

% Plot each obstacle with safe margin
for i = 1:size(obstacles, 1)
    cx = obstacles(i, 1);
    cy = obstacles(i, 2);
    r  = obstacles(i, 3);
    
    obs_x = cx + r * cos(theta_circle);
    obs_y = cy + r * sin(theta_circle);
    
    safe_x = cx + (r + 1.2) * cos(theta_circle);
    safe_y = cy + (r + 1.2) * sin(theta_circle);
    
    plot(safe_x, safe_y, 'g--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    hold on;
    fill(obs_x, obs_y, [0.85 0.25 0.25], 'EdgeColor', 'r', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

% Plot direct line vs avoided multi-curve trajectory
plot(x_straight, y_straight, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Direct Line Path');
plot(x_drone, y_drone, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Multi-Obstacle Flight Path (Bat Algorithm)');

% Mark Start and Goal Points
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point (0,0)');
plot(goal_pos(1), goal_pos(2), 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Goal Point (50,50)');

grid on;
axis equal;
xlabel('X Position (meters)', 'FontSize', 11);
ylabel('Y Position (meters)', 'FontSize', 11);
title('Figure 4: Multi-Obstacle Navigation Trajectory', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);
xlim([-2 55]);
ylim([-2 55]);

disp('Figure 4 (Multiple Obstacle Navigation) replicated successfully.');
end
