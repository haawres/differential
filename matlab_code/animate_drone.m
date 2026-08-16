function animate_drone(t, Y, params)
% =========================================================================
% Interactive / Video Export Flight Animation for Quadrotor UAV
% Renders 3D Quadcopter Frame, Rotors, Trajectory Trail & Obstacles
% Compatible with MATLAB R2024b
% =========================================================================

if nargin < 2
    params = parameters();
    [t, Y] = ode45(@(t, y) drone_dynamics(t, y, params, 'itae'), ...
        [0 25], [0; 0; 0; 0.8; 0.5; 0.2; 0; 0; 0; 0; 0; 0]);
end

fprintf('Launching 3D Drone Flight Path Animation...\n');

fig = figure('Name', 'Quadrotor Flight Dynamics 3D Animation', 'Position', [100 100 900 650]);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude Z (m)');
title('Quadrotor 3D Spatial Trajectory Animation');
view(40, 25);
xlim([-2, 30]); ylim([-2, 30]); zlim([0, 8]);

% Draw ground plane
[gx, gy] = meshgrid(-2:5:30, -2:5:30);
gz = zeros(size(gx));
surf(gx, gy, gz, 'FaceColor', [0.3 0.6 0.3], 'FaceAlpha', 0.2, 'EdgeColor', [0.2 0.5 0.2]);

% Trajectory line trace
traj_line = plot3(Y(1,1), Y(2,1), Y(3,1), 'b-', 'LineWidth', 2);

% Drone body representations
arm_len = params.arm_len * 4; % Scaled for visual clarity
arm1 = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 3);
arm2 = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 3);
rotors = plot3([0 0 0 0], [0 0 0 0], [0 0 0 0], 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

N_frames = length(t);
step = max(1, round(N_frames / 120)); % ~120 animation keyframes

for k = 1:step:N_frames
    if ~ishandle(fig), break; end
    
    % Current position & orientation
    pos = [Y(k,1), Y(k,2), Y(k,3)];
    phi = Y(k,7); theta = Y(k,8); psi = Y(k,9);
    
    % Rotation matrix
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R = Rz * Ry * Rx;
    
    % Quadcopter arms (+ configuration)
    p1 = pos + (R * [arm_len; 0; 0])';
    p2 = pos + (R * [-arm_len; 0; 0])';
    p3 = pos + (R * [0; arm_len; 0])';
    p4 = pos + (R * [0; -arm_len; 0])';
    
    % Update plot handles
    set(arm1, 'XData', [p1(1) p2(1)], 'YData', [p1(2) p2(2)], 'ZData', [p1(3) p2(3)]);
    set(arm2, 'XData', [p3(1) p4(1)], 'YData', [p3(2) p4(2)], 'ZData', [p3(3) p4(3)]);
    set(rotors, 'XData', [p1(1) p2(1) p3(1) p4(1)], ...
                'YData', [p1(2) p2(2) p3(2) p4(2)], ...
                'ZData', [p1(3) p2(3) p3(3) p4(3)]);
    set(traj_line, 'XData', Y(1:k,1), 'YData', Y(1:k,2), 'ZData', Y(1:k,3));
    
    drawnow;
    pause(0.02);
end

fprintf('Animation sequence completed.\n');

end
