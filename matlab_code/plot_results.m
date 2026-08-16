function plot_results(t, YAna, YEuler, YRK4, YODE45)

% Plot Results
%
% This function generates all the comparison plots for the
% numerical methods.
%
% NOTE:
% This is a function file.
% Do NOT run this file directly.
% Run main.m instead.


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