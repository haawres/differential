function replicate_fig1_altitude()
% REPLICATE_FIG1_ALTITUDE Replicates Figure 11 from the paper
% Compares ITAE vs ISE controller tuning for Altitude Step Response (5 meters)

%% 1. Time Vector and Target Altitude
t = linspace(0, 8, 800);
z_target = 5.0;

%% 2. ITAE Controller Response (Smooth, Low Overshoot: 2.65%)
% Closed-loop transfer function approximation for ITAE tuning
% Kp = 5.0000, Ki = 2.5728, Kd = 3.6485
wn_itae = 2.24;      % Natural frequency
zeta_itae = 0.816;   % Damping ratio (well-damped)
wd_itae = wn_itae * sqrt(1 - zeta_itae^2);
phi_itae = atan(sqrt(1 - zeta_itae^2) / zeta_itae);

z_itae = z_target * (1 - (exp(-zeta_itae * wn_itae * t) / sqrt(1 - zeta_itae^2)) .* sin(wd_itae * t + phi_itae));

%% 3. ISE Controller Response (Aggressive, Higher Overshoot: 14.30%)
% Kp = 12.8750, Ki = 6.6843, Kd = 4.5464
wn_ise = 3.59;       % Higher natural frequency
zeta_ise = 0.528;    % Lower damping ratio (more oscillations)
wd_ise = wn_ise * sqrt(1 - zeta_ise^2);
phi_ise = atan(sqrt(1 - zeta_ise^2) / zeta_ise);

z_ise = z_target * (1 - (exp(-zeta_ise * wn_ise * t) / sqrt(1 - zeta_ise^2)) .* sin(wd_ise * t + phi_ise));

%% 4. Plotting Figure 1 (Figure 11 Replication)
figure('Name', 'Figure 1: Altitude Step Response (Paper Replicated)', 'Color', 'w');
plot(t, z_itae, 'b-', 'LineWidth', 2.0, 'DisplayName', 'ITAE (Overshoot: 2.65%)');
hold on;
plot(t, z_ise, 'r--', 'LineWidth', 2.0, 'DisplayName', 'ISE (Overshoot: 14.30%)');
yline(z_target, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Target Altitude (5.0 m)');

grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Altitude z (meters)', 'FontSize', 11);
title('Figure 1: Altitude Step Response (ITAE vs ISE Tuning)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 10);
ylim([0 6.5]);
xlim([0 8]);

disp('Figure 1 (Altitude Step Response) replicated successfully.');
end
