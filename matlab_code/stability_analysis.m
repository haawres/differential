function stability_analysis(params)
% Drone Flight Simulation Project
% Stability Analysis
% This function investigates how the Euler method behaves when different step sizes are used.

% Step sizes to test
stepSizes = [0.1 0.5 1.0];

% Line styles
styles = {'b','g','r'};

figure('Name','Euler Stability Analysis',...
    'NumberTitle','off',...
    'Position',[100 100 900 600]);

hold on

for k = 1:length(stepSizes)

    % Copy parameters
    p = params;

    % Change step size
    p.h = stepSizes(k);

    % Analytical solution
    [tAna, YAna] = analytical_solution(p);

    % Euler solution
    [tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics,p);

    % Position error
    error = sqrt( ...
        (YAna(1,:)-YEuler(1,:)).^2 + ...
        (YAna(2,:)-YEuler(2,:)).^2 + ...
        (YAna(3,:)-YEuler(3,:)).^2 );

    plot(tAna,error,...
        styles{k},...
        'LineWidth',2,...
        'DisplayName',['h = ' num2str(stepSizes(k))]);

end

grid on
grid minor
box on

xlabel('Time (s)','FontSize',12)
ylabel('Position Error (m)','FontSize',12)

title('Euler Stability for Different Step Sizes','FontSize',14)

legend('Location','northwest')

end