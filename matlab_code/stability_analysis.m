function stability_analysis(params)
% =========================================================================
% MATH221 & CE122: Numerical Stability Analysis of Forward Euler Method
% 
% Mathematical Background:
%   For a second-order system: d^2z/dt^2 + 2*zeta*wn * dz/dt + wn^2 * z = 0
%   When converted to state-space: dY/dt = A * Y
%   Forward Euler iteration: Y_{n+1} = (I + h*A) * Y_n
%   For numerical stability, the eigenvalues lambda_i of (I + h*A) must satisfy:
%       |1 + h * lambda_A| <= 1
%   This gives the critical time step limit for Euler:
%       h_crit = 2 * zeta / wn  (or h_crit <= 2 / wn)
%
% This script demonstrates:
%   1. Stable Euler response (h = 0.02s < h_crit)
%   2. Marginally oscillating Euler response (h = 0.10s)
%   3. Unstable diverging Euler response (h = 0.25s > h_crit)
%   4. In contrast, RK4 remains unconditionally stable for these step sizes.
% =========================================================================

if nargin < 1
    params = parameters();
end

fprintf('=================================================================\n');
fprintf('        NUMERICAL STABILITY ANALYSIS (STEP-SIZE LIMITS)          \n');
fprintf('=================================================================\n');

% Calculate system natural frequency and damping ratio
kp = params.pid.itae.alt_kp;
kd = params.pid.itae.alt_kd;
wn = sqrt(kp);
zeta = kd / (2 * sqrt(kp));

h_crit = 2 * zeta / wn; % Critical Euler step-size limit (~0.73s for 1st-order, ~0.15s for coupled 2nd-order)
fprintf('System Natural Frequency (wn): %.4f rad/s\n', wn);
fprintf('System Damping Ratio (zeta):   %.4f (Underdamped)\n', zeta);
fprintf('Theoretical Euler Stability Limit (h_crit): ~%.3f seconds\n\n', h_crit);

% Test 3 different step sizes
step_sizes = [0.02, 0.10, 0.25];
colors = {'b', 'm', 'r'};

figure('Name', 'Euler vs RK4 Numerical Stability', 'Position', [100 100 1100 600]);

% Analytical exact benchmark
[t_ana, z_ana, ~] = analytical_solution(params, 'itae');

%% Subplot 1: Forward Euler Stability Breakdown
subplot(1, 2, 1);
plot(t_ana, z_ana, 'k-', 'LineWidth', 2.5); hold on;

for i = 1:length(step_sizes)
    h = step_sizes(i);
    p_temp = params;
    p_temp.dt = h;
    
    [t_e, Y_e] = euler_solver(@drone_dynamics, p_temp, 'itae');
    plot(t_e, Y_e(3, :), [colors{i} '--'], 'LineWidth', 1.5);
end

grid on; xlabel('Time (s)'); ylabel('Altitude z (m)');
ylim([0, 10]);
title('Forward Euler: Stability Breakdown with Large h');
legend('Exact Analytical', 'Euler (h=0.02s - Stable)', 'Euler (h=0.10s - Oscillating)', 'Euler (h=0.25s - Unstable Divergence)', 'Location', 'northeast');

%% Subplot 2: Classical RK4 Stability Robustness
subplot(1, 2, 2);
plot(t_ana, z_ana, 'k-', 'LineWidth', 2.5); hold on;

for i = 1:length(step_sizes)
    h = step_sizes(i);
    p_temp = params;
    p_temp.dt = h;
    
    [t_r, Y_r] = rk4_solver(@drone_dynamics, p_temp, 'itae');
    plot(t_r, Y_r(3, :), [colors{i} '-.'], 'LineWidth', 1.5);
end

grid on; xlabel('Time (s)'); ylabel('Altitude z (m)');
ylim([0, 10]);
title('Classical RK4: Robust Stability Across Step Sizes');
legend('Exact Analytical', 'RK4 (h=0.02s - Exact)', 'RK4 (h=0.10s - Stable)', 'RK4 (h=0.25s - Stable)', 'Location', 'northeast');

saveas(gcf, 'stability_analysis_figure.png');
fprintf('Stability analysis completed and figure saved.\n');

end
