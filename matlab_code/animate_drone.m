function animate_drone(Y, params)
% ANIMATE_DRONE Renders flight path animation of the drone around an obstacle.

fig = figure('Name','Drone Flight Animation',...
       'NumberTitle','off',...
       'Position',[120 80 1000 650],...
       'Color','w');

hold on;
grid on;
grid minor;
box on;
axis equal;

xlabel('x Position (m)', 'FontSize', 11);
ylabel('y Position (m)', 'FontSize', 11);
title('Drone Flight Animation', 'FontSize', 12, 'FontWeight', 'bold');

%% Obstacle 
theta = linspace(0, 2*pi, 200);

plot(150 + 20*cos(theta), -70 + 20*sin(theta), 'k', 'LineWidth', 2);
plot(150, -70, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(153, -68, 'Obstacle', 'FontWeight', 'bold');

%% Planned Path 
plot(Y(1,:), Y(2,:), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1.5);

%% Start & Finish 
plot(Y(1,1), Y(2,1), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
text(Y(1,1)+5, Y(2,1)-2, 'Start', 'FontWeight', 'bold');

plot(Y(1,end), Y(2,end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(Y(1,end)-18, Y(2,end)-8, 'Finish', 'FontWeight', 'bold');

%% Axis Limits
xlim([0 max(Y(1,:))+20]);
ylim([min(Y(2,:))-20 20]);

%% Flight Trail
trail = animatedline('Color', 'b', 'LineWidth', 2);

%% Drone Body
L = 5;
arm1 = plot([0 0], [0 0], 'r', 'LineWidth', 2);
arm2 = plot([0 0], [0 0], 'r', 'LineWidth', 2);
body = plot(0, 0, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 9);

%% Time Display
dt_val = 0.1;
if isfield(params, 'dt')
    dt_val = params.dt;
elseif isfield(params, 'h')
    dt_val = params.h;
end

timeText = text(10, 16, 'Time = 0.0 s',...
    'FontSize', 12,...
    'FontWeight', 'bold',...
    'BackgroundColor', 'white');

%% Animation Loop (Sampled for smooth playback & publishing compatibility)
num_points = size(Y, 2);
step = max(1, round(num_points / 120)); % Smooth 120 frame sample

for k = 1:step:num_points
    if ~isvalid(fig) || ~isvalid(trail)
        break;
    end

    x = Y(1,k);
    y = Y(2,k);

    % Update trail
    addpoints(trail, x, y);

    % Update drone arms and body
    set(arm1, 'XData', [x-L, x+L], 'YData', [y, y]);
    set(arm2, 'XData', [x, x], 'YData', [y-L, y+L]);
    set(body, 'XData', x, 'YData', y);

    % Update time
    set(timeText, 'String', sprintf('Time = %.1f s', (k-1)*dt_val));

    drawnow limitrate;
    pause(0.01);
end

% Ensure final frame shows complete flight path
if isvalid(trail) && isvalid(fig)
    addpoints(trail, Y(1,end), Y(2,end));
    set(arm1, 'XData', [Y(1,end)-L, Y(1,end)+L], 'YData', [Y(2,end), Y(2,end)]);
    set(arm2, 'XData', [Y(1,end), Y(1,end)], 'YData', [Y(2,end)-L, Y(2,end)+L]);
    set(body, 'XData', Y(1,end), 'YData', Y(2,end));
    set(timeText, 'String', sprintf('Time = %.1f s', (num_points-1)*dt_val));
    drawnow;
end

end