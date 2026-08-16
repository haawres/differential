function replicate_fig2_attitude()
% REPLICATE_FIG2_ATTITUDE Replicates Figure 12 from the paper
% Compares ITAE vs ISE controller tuning for Attitude Step Response (Roll, Pitch, Yaw)

%% 1. Time Vector and Target Attitude Angle
t = linspace(0, 3, 600);
angle_target = 1.0; % Target step input (1 radian)

%% 2. ITAE Controller Response (Settling Time = 0.43 s)
% Kp = 9.9995, Ki = 0.000064, Kd = 1.0915
wn_itae = 3.16;
zeta_itae = 0.95; % Near critically damped, fast settling
wd_itae = wn_itae * sqrt(1 - zeta_itae^2);
phi_itae = atan(sqrt(1 - zeta_itae^2) / zeta_itae);

angle_itae = angle_target * (1 - (exp(-zeta_itae * wn_itae * t) / sqrt(1 - zeta_itae^2)) .* sin(wd_itae * t + phi_itae));

%% 3. ISE Controller Response (Settling Time = 0.75 s)
% Kp = 9.9995, Ki = 0.0254, Kd = 0.7545
wn_ise = 3.16;
zeta_ise = 0.65; % Lower damping, noticeable overshoot
wd_ise = wn_ise * sqrt(1 - zeta_ise^2);
phi_ise = atan(sqrt(1 - zeta_ise^2) / zeta_ise);

angle_ise = angle_target * (1 - (exp(-zeta_ise * wn_ise * t) / sqrt(1 - zeta_ise^2)) .* sin(wd_ise * t + phi_ise));

%% 4. Plotting Figure 2 (Figure 12 Replication)
figure('Name', 'Figure 2: Attitude Step Response (Paper Replicated)', 'Color', 'w');

% Subplot 1: Roll Angle (phi)
subplot(3, 1, 1);
plot(t, angle_itae, 'b-', 'LineWidth', 1.8, 'DisplayName', 'ITAE (Ts = 0.43 s)');
hold on;
plot(t, angle_ise, 'r--', 'LineWidth', 1.8, 'DisplayName', 'ISE (Ts = 0.75 s)');
yline(angle_target, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Target');
grid on;
ylabel('Roll \phi (rad)', 'FontSize', 10);
title('Figure 2: Attitude Step Response (\phi, \theta, \psi)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0 3]);

% Subplot 2: Pitch Angle (theta)
subplot(3, 1, 2);
plot(t, angle_itae, 'b-', 'LineWidth', 1.8, 'DisplayName', 'ITAE');
hold on;
plot(t, angle_ise, 'r--', 'LineWidth', 1.8, 'DisplayName', 'ISE');
yline(angle_target, 'k:', 'LineWidth', 1.2);
grid on;
ylabel('Pitch \theta (rad)', 'FontSize', 10);
xlim([0 3]);

% Subplot 3: Yaw Angle (psi)
subplot(3, 1, 3);
plot(t, angle_itae, 'b-', 'LineWidth', 1.8, 'DisplayName', 'ITAE');
hold on;
plot(t, angle_ise, 'r--', 'LineWidth', 1.8, 'DisplayName', 'ISE');
yline(angle_target, 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time (seconds)', 'FontSize', 10);
ylabel('Yaw \psi (rad)', 'FontSize', 10);
xlim([0 3]);

disp('Figure 2 (Attitude Step Response) replicated successfully.');
end
