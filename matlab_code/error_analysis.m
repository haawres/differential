function error_analysis(params)
% =========================================================================
% Applied Programming Requirement: Comprehensive Numerical Error Analysis
% Investigates:
%   1. Step-size convergence (Euler O(h) vs RK4 O(h^4)) on log-log scale
%   2. Tracking error metrics (RMSE, MAE, Max Deviation)
%   3. Absolute error distributions across flight trajectories
% =========================================================================

if nargin < 1
    params = parameters();
end

fprintf('Executing Numerical Error & Step-Size Convergence Analysis...\n');

step_sizes = [0.2, 0.1, 0.05, 0.02, 0.01, 0.005];
N_steps = length(step_sizes);

euler_rmse = zeros(1, N_steps);
rk4_rmse   = zeros(1, N_steps);

for i = 1:N_steps
    h = step_sizes(i);
    p_temp = params;
    p_temp.dt = h;
    
    [t_ana, z_ana, ~] = analytical_solution(p_temp, 'itae');
    [~, Y_e] = euler_solver(@drone_dynamics, p_temp, 'itae');
    [~, Y_r] = rk4_solver(@drone_dynamics, p_temp, 'itae');
    
    z_e = Y_e(3, :);
    z_r = Y_r(3, :);
    
    euler_rmse(i) = sqrt(mean((z_e - z_ana).^2));
    rk4_rmse(i)   = sqrt(mean((z_r - z_ana).^2));
end

%% Plot Convergence Order (Log-Log Plot)
figure('Name', 'Numerical Convergence & Error Analysis', 'Position', [150 150 1000 600]);

subplot(1, 2, 1);
loglog(step_sizes, euler_rmse, 'ro-', 'LineWidth', 2, 'MarkerSize', 8); hold on;
loglog(step_sizes, rk4_rmse, 'bs-', 'LineWidth', 2, 'MarkerSize', 8);
% Reference slopes
loglog(step_sizes, step_sizes * (euler_rmse(1)/step_sizes(1)), 'r--', 'LineWidth', 1.2);
loglog(step_sizes, (step_sizes.^4) * (rk4_rmse(1)/(step_sizes(1)^4)), 'b--', 'LineWidth', 1.2);
grid on; xlabel('Step Size h (s)'); ylabel('RMSE vs Analytical (m)');
title('Step-Size Convergence Study (Log-Log)');
legend('Euler RMSE', 'RK4 RMSE', 'Theoretical Slope O(h)', 'Theoretical Slope O(h^4)', 'Location', 'northwest');

subplot(1, 2, 2);
[t_full, z_full, ~] = analytical_solution(params, 'itae');
[~, Y_full_e] = euler_solver(@drone_dynamics, params, 'itae');
[~, Y_full_r] = rk4_solver(@drone_dynamics, params, 'itae');

err_e = abs(Y_full_e(3,:) - z_full);
err_r = abs(Y_full_r(3,:) - z_full);

plot(t_full, err_e, 'r-', 'LineWidth', 1.5); hold on;
plot(t_full, err_r, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Absolute Error (m)');
title('Absolute Error Time Distribution (dt = 0.02s)');
legend('Euler Absolute Error', 'RK4 Absolute Error', 'Location', 'northeast');

saveas(gcf, 'error_convergence_figure.png');
fprintf('Error analysis completed and plots generated successfully.\n');

end
