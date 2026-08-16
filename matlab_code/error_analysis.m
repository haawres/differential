function error_analysis(t, YAna, YEuler, YRK4, YODE45)

% Error Analysis
%
% Compares the numerical methods against the analytical
% solution by plotting the absolute error in velocity.
%
% NOTE:
% This is a function file.
% Do NOT run this file directly.


%% Calculate Absolute Errors

eEuler = abs(YAna(4:6,:) - YEuler(4:6,:));
eRK4   = abs(YAna(4:6,:) - YRK4(4:6,:));
eODE45 = abs(YAna(4:6,:) - YODE45(4:6,:));

titles = {'Absolute Error in Forward Velocity (v_x)','Absolute Error in Lateral Velocity (v_y)','Absolute Error in Vertical Velocity (v_z)'};

ylabels = {'|Error in v_x| (m/s)','|Error in v_y| (m/s)','|Error in v_z| (m/s)'};

figure( ...
    'Name','Error Analysis', ...
    'NumberTitle','off', ...
    'Position', [100 100 950 750]);

for i = 1:3

    subplot(3,1,i)

    plot(t, zeros(size(t)), 'k', 'LineWidth', 2)
    hold on

    plot(t,eEuler(i,:),'r--','LineWidth', 2)
    hold on

    plot(t,eRK4(i,:),'g-.','LineWidth', 2)

    plot(t,eODE45(i,:),'b:','LineWidth',2)

    grid minor
    box on

    title(titles{i})
    xlabel('Time (s)')
    ylabel(ylabels{i})

    legend('Analytical (Reference)',...
        'Euler', ...
        'RK4', ...
        'ode45', ...
        'Location','best');

end

end