function sensitivity_analysis(params)
% =========================================================================
% Differential Equations Requirement: Parameter Sensitivity Sweep (OAT)
% Investigates +/-10% and +/-20% perturbations across 8 core parameters:
%   - Mass (m), Arm Length (l), Inertia Ix, Iz, Thrust b, Drag d, Kp, Kd
% Measures: Overshoot (%), Settling Time (s), Rise Time (s), Steady-State Error
% Generates: Sensitivity Indices & Tornado Ranking Chart
% =========================================================================

if nargin < 1
    params = parameters();
end

fprintf('=================================================================\n');
fprintf('        SYSTEMATIC PARAMETER SENSITIVITY SWEEP (+/- 20%%)        \n');
fprintf('=================================================================\n');

param_names = {'Mass (m)', 'Inertia (Ix)', 'Inertia (Iz)', 'Thrust Coeff (b)', ...
               'Drag Coeff (d)', 'Arm Length (l)', 'Altitude Kp', 'Altitude Kd'};
num_params = length(param_names);

% Perturbation factors
pert_factors = [0.8, 0.9, 1.0, 1.1, 1.2]; % -20%, -10%, Baseline, +10%, +20%

% Baseline metrics
[t_base, z_base, ~] = analytical_solution(params, 'itae');
os_base = calculate_overshoot(z_base, params.target_z);
st_base = calculate_settling_time(t_base, z_base, params.target_z);

sens_matrix_os = zeros(num_params, length(pert_factors));
sens_matrix_st = zeros(num_params, length(pert_factors));

for p = 1:num_params
    for f = 1:length(pert_factors)
        factor = pert_factors(f);
        p_mod = params;
        
        switch p
            case 1, p_mod.mass = params.mass * factor;
            case 2, p_mod.Ix = params.Ix * factor;
            case 3, p_mod.Iz = params.Iz * factor;
            case 4, p_mod.thrust_b = params.thrust_b * factor;
            case 5, p_mod.drag_d = params.drag_d * factor;
            case 6, p_mod.arm_len = params.arm_len * factor;
            case 7, p_mod.pid.itae.alt_kp = params.pid.itae.alt_kp * factor;
            case 8, p_mod.pid.itae.alt_kd = params.pid.itae.alt_kd * factor;
        end
        
        [t_sim, z_sim, ~] = analytical_solution(p_mod, 'itae');
        sens_matrix_os(p, f) = calculate_overshoot(z_sim, params.target_z);
        sens_matrix_st(p, f) = calculate_settling_time(t_sim, z_sim, params.target_z);
    end
end

%% Print Formatted Table
fprintf('%-18s | %-8s | %-8s | %-8s | %-8s | %-8s\n', 'Parameter', '-20%', '-10%', 'Baseline', '+10%', '+20%');
fprintf('-----------------------------------------------------------------\n');
for p = 1:num_params
    fprintf('%-18s | %-8.2f | %-8.2f | %-8.2f | %-8.2f | %-8.2f  (Overshoot %%)\n', ...
        param_names{p}, sens_matrix_os(p,1), sens_matrix_os(p,2), sens_matrix_os(p,3), sens_matrix_os(p,4), sens_matrix_os(p,5));
end
fprintf('=================================================================\n');

%% Tornado Ranking Chart
figure('Name', 'Parameter Sensitivity Tornado Ranking', 'Position', [100 100 1100 600]);

subplot(1, 2, 1);
delta_os_neg = sens_matrix_os(:, 1) - sens_matrix_os(:, 3); % -20% delta
delta_os_pos = sens_matrix_os(:, 5) - sens_matrix_os(:, 3); % +20% delta
[~, sort_idx] = sort(abs(delta_os_pos) + abs(delta_os_neg), 'ascend');

b = barh(categorical(param_names(sort_idx)), [delta_os_neg(sort_idx), delta_os_pos(sort_idx)]);
b(1).FaceColor = [0.9 0.3 0.3];
b(2).FaceColor = [0.2 0.7 0.4];
grid on; xlabel('Change in Overshoot (\Delta %)');
title('Tornado Chart: Sensitivity to Overshoot');
legend('-20% Perturbation', '+20% Perturbation', 'Location', 'southeast');

subplot(1, 2, 2);
delta_st_neg = sens_matrix_st(:, 1) - sens_matrix_st(:, 3);
delta_st_pos = sens_matrix_st(:, 5) - sens_matrix_st(:, 3);
[~, sort_idx_st] = sort(abs(delta_st_pos) + abs(delta_st_neg), 'ascend');

b_st = barh(categorical(param_names(sort_idx_st)), [delta_st_neg(sort_idx_st), delta_st_pos(sort_idx_st)]);
b_st(1).FaceColor = [0.9 0.5 0.2];
b_st(2).FaceColor = [0.3 0.6 0.9];
grid on; xlabel('Change in Settling Time (\Delta s)');
title('Tornado Chart: Sensitivity to Settling Time');
legend('-20% Perturbation', '+20% Perturbation', 'Location', 'southeast');

saveas(gcf, 'tornado_sensitivity_figure.png');
fprintf('Tornado sensitivity chart generated.\n');

end

%% Helper Functions
function os = calculate_overshoot(z, z_target)
    z_max = max(z);
    if z_max > z_target
        os = ((z_max - z_target) / z_target) * 100.0;
    else
        os = 0.0;
    end
end

function st = calculate_settling_time(t, z, z_target)
    band = 0.02 * z_target; % 2% error band
    idx = find(abs(z - z_target) > band, 1, 'last');
    if isempty(idx)
        st = t(1);
    else
        st = t(idx);
    end
end
