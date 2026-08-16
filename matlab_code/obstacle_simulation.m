function obstacle_simulation(params)
% Runs the obstacle avoidance simulation

% Analytical solution is not used here because of the obstacle.

[tEuler,YEuler] = euler_solver(@drone_obstacle_dynamics,params);

[tRK4,YRK4] = rk4_solver(@drone_obstacle_dynamics,params);

[tODE45,YODE45] = ode45_solver(@drone_obstacle_dynamics,params);

figure('Name','Obstacle Avoidance',...
    'NumberTitle','off',...
    'Position',[100 100 900 650]);

plot(YEuler(1,:),YEuler(2,:),'r--','LineWidth',2)
hold on

plot(YRK4(1,:),YRK4(2,:),'g-.','LineWidth',2)

plot(YODE45(1,:),YODE45(2,:),'b','LineWidth',2)

% Draw obstacle
thetaCircle = linspace(0,2*pi,200);

plot(150+20*cos(thetaCircle),...
    -70+20*sin(thetaCircle),...
    'k','LineWidth',2)

grid on
axis equal

xlabel('x Position (m)')
ylabel('y Position (m)')

title('Obstacle Avoidance Trajectory')
plot(150,-70,'ko',...
    'MarkerFaceColor','k',...
    'MarkerSize',8)

text(153,-68,'Obstacle',...
    'FontSize',10,...
    'FontWeight','bold')

legend('Euler',...
    'RK4',...
    'ode45',...
    'Obstacle',...
    'Location','best')

end