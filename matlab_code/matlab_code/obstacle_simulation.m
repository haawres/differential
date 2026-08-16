function obstacle_simulation(params)
% =========================================================================
% Obstacle Avoidance Simulation & Trajectory Generation
% Replicates: Elite Opposition-Based Bat Algorithm (EOBA) & LiDAR thresholds
% Compares: Safe 1.0m threshold vs Unsafe 0.5m collision boundary
% =========================================================================

if nargin < 1
    params = parameters();
end

fprintf('Generating 2D/3D Obstacle Avoidance Trajectories...\n');

% Obstacle locations [x, y, radius]
obstacles = [10, 10, 1.0;
             25, 20, 1.2;
             40, 15, 1.0];

% Safe Trajectory (1.0m Warning Threshold)
[t_safe, Y_safe] = ode45(@(t, y) drone_modified_dynamics(t, y, params, 'itae'), ...
    [0 40], [0; 0; 5; 1.2; 0.8; 0; 0; 0; 0; 0; 0; 0]);

% Unsafe Trajectory (No avoidance / Low threshold with disturbance)
p_dist = params;
p_dist.wind_x = 0.8;
[t_dist, Y_dist] = ode45(@(t, y) drone_modified_dynamics(t, y, p_dist, 'ise'), ...
    [0 40], [0; 0; 5; 1.2; 0.8; 0; 0; 0; 0; 0; 0; 0]);

figure('Name', 'Obstacle Avoidance Flight Paths', 'Position', [150 150 1000 600]);

subplot(1, 2, 1);
% Plot Obstacles
theta = linspace(0, 2*pi, 100);
for i = 1:size(obstacles, 1)
    ox = obstacles(i,1) + obstacles(i,3)*cos(theta);
    oy = obstacles(i,2) + obstacles(i,3)*sin(theta);
    fill(ox, oy, [0.85 0.35 0.35], 'FaceAlpha', 0.6, 'EdgeColor', 'r', 'LineWidth', 1.5); hold on;
    plot(obstacles(i,1), obstacles(i,2), 'k+', 'LineWidth', 2);
end

plot(Y_safe(:,1), Y_safe(:,2), 'b-', 'LineWidth', 2.5);
plot(Y_dist(:,1), Y_dist(:,2), 'r--', 'LineWidth', 2.0);
plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(45, 25, 'mp', 'MarkerFaceColor', 'm', 'MarkerSize', 12);
grid on; xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Horizontal Avoidance Trajectory (2D Plane)');
legend('Obstacle Boundary', 'Obstacle Center', '', '', '', '', ...
       'ITAE + 1.0m Safety Margin', 'ISE + Wind Disturbance (Collision Drift)', ...
       'Start Point', 'Goal Target', 'Location', 'northwest');
axis equal;

subplot(1, 2, 2);
plot3(Y_safe(:,1), Y_safe(:,2), Y_safe(:,3), 'b-', 'LineWidth', 2.5); hold on;
plot3(Y_dist(:,1), Y_dist(:,2), Y_dist(:,3), 'r--', 'LineWidth', 2.0);
for i = 1:size(obstacles, 1)
    [cx, cy, cz] = cylinder(obstacles(i,3), 30);
    cz = cz * 6; % Height of obstacle cylinder
    surf(cx + obstacles(i,1), cy + obstacles(i,2), cz, 'FaceColor', [0.8 0.4 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude Z (m)');
title('3D UAV Navigation Pipeline');
view(45, 30);

saveas(gcf, 'obstacle_avoidance_trajectory.png');
fprintf('Obstacle avoidance simulation plots exported.\n');

end
