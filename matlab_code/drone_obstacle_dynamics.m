function dY = drone_obstacle_dynamics(t, Y, params)
% Drone dynamics with obstacle avoidance.
% When the drone gets close to the obstacle, a sideways acceleration is applied to steer around it.


% State variables
x  = Y(1);
y  = Y(2);
z  = Y(3);

vx = Y(4);
vy = Y(5);
vz = Y(6);

% Parameters
m = params.mass;
T = params.thrust;
k = params.drag;
g = params.gravity;

theta = params.pitch;
phi = params.roll;

% Normal accelerations
ax = (T/m)*sin(theta)*cos(phi) - (k/m)*vx;
ay = -(T/m)*sin(phi) - (k/m)*vy;
az = (T/m)*cos(theta)*cos(phi) - g - (k/m)*vz;

%% Obstacle location
obstacleX = 150;
obstacleY = -70;
obstacleRadius = 20;

% Distance from obstacle
distance = sqrt((x-obstacleX)^2 + (y-obstacleY)^2);

%% Avoidance maneuver
if distance < obstacleRadius

    % Stronger avoidance as the drone gets closer
    avoidanceStrength = 8 * (1 - distance/obstacleRadius);

    ay = ay + avoidanceStrength;

end

% State derivatives
dY = [
    vx;
    vy;
    vz;
    ax;
    ay;
    az
    ];

end