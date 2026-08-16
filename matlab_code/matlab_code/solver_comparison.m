function results = solver_comparison(params)
% =========================================================================
% Applied Programming Requirement: Comprehensive ODE Solver Benchmarking
% Evaluates: ode45, ode23, ode15s, ode113, RK4, and Forward Euler
% Compares: Accuracy (RMSE, Max Error), Step Counts, Computational Runtime
% =========================================================================

if nargin < 1
    params = parameters();
end

fprintf('=================================================================\n');
fprintf('       COMPREHENSIVE ODE NUMERICAL SOLVER BENCHMARK REPORT       \n');
fprintf('=================================================================\n');

% Analytical benchmark for altitude
[t_ana, z_ana, ~] = analytical_solution(params, 'itae');

%% 1. ode45 Benchmark (Dormand-Prince 4/5)
tic;
[t_ode45, Y_ode45] = ode45(@(t,y) drone_dynamics(t,y,params,'itae'), ...
    [params.t0 params.tf], params.Y0, odeset('RelTol',1e-6,'AbsTol',1e-8));
time_ode45 = toc * 1000;
z_ode45_interp = interp1(t_ode45, Y_ode45(:,3), t_ana);
rmse_ode45 = sqrt(mean((z_ode45_interp - z_ana).^2));
max_err_ode45 = max(abs(z_ode45_interp - z_ana));

%% 2. ode23 Benchmark (Bogacki-Shampine 2/3)
tic;
[t_ode23, Y_ode23] = ode23(@(t,y) drone_dynamics(t,y,params,'itae'), ...
    [params.t0 params.tf], params.Y0, odeset('RelTol',1e-6,'AbsTol',1e-8));
time_ode23 = toc * 1000;
z_ode23_interp = interp1(t_ode23, Y_ode23(:,3), t_ana);
rmse_ode23 = sqrt(mean((z_ode23_interp - z_ana).^2));
max_err_ode23 = max(abs(z_ode23_interp - z_ana));

%% 3. ode15s Benchmark (Stiff BDF/NDF)
tic;
[t_ode15s, Y_ode15s] = ode15s(@(t,y) drone_dynamics(t,y,params,'itae'), ...
    [params.t0 params.tf], params.Y0, odeset('RelTol',1e-6,'AbsTol',1e-8));
time_ode15s = toc * 1000;
z_ode15s_interp = interp1(t_ode15s, Y_ode15s(:,3), t_ana);
rmse_ode15s = sqrt(mean((z_ode15s_interp - z_ana).^2));
max_err_ode15s = max(abs(z_ode15s_interp - z_ana));

%% 4. ode113 Benchmark (Adams-Bashforth-Moulton)
tic;
[t_ode113, Y_ode113] = ode113(@(t,y) drone_dynamics(t,y,params,'itae'), ...
    [params.t0 params.tf], params.Y0, odeset('RelTol',1e-6,'AbsTol',1e-8));
time_ode113 = toc * 1000;
z_ode113_interp = interp1(t_ode113, Y_ode113(:,3), t_ana);
rmse_ode113 = sqrt(mean((z_ode113_interp - z_ana).^2));
max_err_ode113 = max(abs(z_ode113_interp - z_ana));

%% 5. Classical RK4 Benchmark (Fixed Step dt=0.02s)
tic;
[t_rk4, Y_rk4] = rk4_solver(@drone_dynamics, params, 'itae');
time_rk4 = toc * 1000;
z_rk4 = Y_rk4(3, :);
rmse_rk4 = sqrt(mean((z_rk4 - z_ana).^2));
max_err_rk4 = max(abs(z_rk4 - z_ana));

%% 6. Forward Euler Benchmark (Fixed Step dt=0.02s)
tic;
[t_euler, Y_euler] = euler_solver(@drone_dynamics, params, 'itae');
time_euler = toc * 1000;
z_euler = Y_euler(3, :);
rmse_euler = sqrt(mean((z_euler - z_ana).^2));
max_err_euler = max(abs(z_euler - z_ana));

%% Display Formatted Results Table
fprintf('%-12s | %-12s | %-12s | %-12s | %-10s\n', ...
    'Solver', 'Steps', 'Runtime (ms)', 'RMSE (m)', 'Max Err (m)');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'ode45', length(t_ode45), time_ode45, rmse_ode45, max_err_ode45);
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'ode23', length(t_ode23), time_ode23, rmse_ode23, max_err_ode23);
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'ode15s', length(t_ode15s), time_ode15s, rmse_ode15s, max_err_ode15s);
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'ode113', length(t_ode113), time_ode113, rmse_ode113, max_err_ode113);
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'RK4 (Fixed)', length(t_rk4), time_rk4, rmse_rk4, max_err_rk4);
fprintf('%-12s | %-12d | %-12.2f | %-12.4e | %-10.4e\n', 'Euler (Fixed)', length(t_euler), time_euler, rmse_euler, max_err_euler);
fprintf('=================================================================\n');

%% Plot Visual Comparison Figure
figure('Name', 'ODE Solver Benchmark Comparison', 'Position', [100 100 1100 700]);

subplot(2, 2, 1);
plot(t_ana, z_ana, 'k-', 'LineWidth', 2.5); hold on;
plot(t_ode45, Y_ode45(:,3), 'b--', 'LineWidth', 1.5);
plot(t_euler, z_euler, 'r:', 'LineWidth', 1.5);
plot(t_rk4, z_rk4, 'g-.', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Step Response (All Solvers)');
legend('Analytical', 'ode45', 'Euler (h=0.02s)', 'RK4 (h=0.02s)', 'Location', 'southeast');

subplot(2, 2, 2);
semilogy(t_ana, abs(z_ode45_interp - z_ana) + 1e-12, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t_ana, abs(z_ode23_interp - z_ana) + 1e-12, 'c--', 'LineWidth', 1.5);
semilogy(t_ana, abs(z_rk4 - z_ana) + 1e-12, 'g-.', 'LineWidth', 1.5);
semilogy(t_ana, abs(z_euler - z_ana) + 1e-12, 'r:', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Absolute Error |z - z_{ana}| (m)');
title('Logarithmic Error Over Time');
legend('ode45', 'ode23', 'RK4', 'Euler', 'Location', 'northeast');

subplot(2, 2, 3);
solvers = {'ode45', 'ode23', 'ode15s', 'ode113', 'RK4', 'Euler'};
runtimes = [time_ode45, time_ode23, time_ode15s, time_ode113, time_rk4, time_euler];
b1 = bar(categorical(solvers), runtimes, 'FaceColor', [0.2 0.6 0.9]);
grid on; ylabel('Computational Time (ms)');
title('Execution Runtime Comparison');

subplot(2, 2, 4);
rmses = [rmse_ode45, rmse_ode23, rmse_ode15s, rmse_ode113, rmse_rk4, rmse_euler];
b2 = bar(categorical(solvers), rmses, 'FaceColor', [0.9 0.4 0.3]);
set(gca, 'YScale', 'log');
grid on; ylabel('Root Mean Square Error (m) [Log Scale]');
title('Solver Accuracy (RMSE vs Analytical)');

saveas(gcf, 'solver_comparison_figure.png');

results.ode45.rmse = rmse_ode45;
results.ode45.time = time_ode45;
results.rk4.rmse = rmse_rk4;
results.euler.rmse = rmse_euler;

end
