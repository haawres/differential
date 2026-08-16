%% =========================================================================
% MATH221 & CE122: Applied Differential Equations & Numerical Methods
% Drone Dynamics & Control Project - Master Simulation & Analysis Script
% Authors: KEMS UAV Research Group (Ashesi University)
% Reference: Alanezi et al., MDPI Drones 2022, 6, 288
% Compatible with MATLAB R2024b
% =========================================================================

clear; clc; close all;

fprintf('*****************************************************************\n');
fprintf('*        KEMS UAV DRONE DYNAMICS & CONTROL SIMULATION           *\n');
fprintf('*        MATH221 Differential Equations & Numerical Methods     *\n');
fprintf('*****************************************************************\n\n');

%% 1. Initialize System Parameters
fprintf('[Step 1/7] Initializing physical parameters and GA-PID gains...\n');
params = parameters();
fprintf('-> Mass: %.3f kg, Arm: %.3f m, Target Altitude: %.1f m\n', ...
    params.mass, params.arm_len, params.target_z);

%% 2. Calculate Analytical Benchmark & Standard ODE Solutions
fprintf('\n[Step 2/7] Calculating analytical benchmark vs numerical ODEs...\n');
[t_ana, z_ana, vz_ana] = analytical_solution(params, 'itae');
[t_ode45, Y_ode45]     = ode45_solver(@drone_dynamics, params, 'itae');
[t_rk4, Y_rk4]         = rk4_solver(@drone_dynamics, params, 'itae');
[t_euler, Y_euler]     = euler_solver(@drone_dynamics, params, 'itae');

fprintf('-> Completed baseline simulations for ITAE Altitude Step Response.\n');

%% 3. Solver Comparison & Benchmarking
fprintf('\n[Step 3/7] Running Multi-Solver Benchmarking (ode45, ode23, ode15s, ode113, RK4, Euler)...\n');
solver_results = solver_comparison(params);

%% 4. Numerical Error & Convergence Analysis
fprintf('\n[Step 4/7] Executing Step-Size Convergence & Error Analysis...\n');
error_analysis(params);

%% 5. Numerical Stability Analysis (Step-Size Limits)
fprintf('\n[Step 5/7] Executing Euler vs RK4 Numerical Stability Analysis...\n');
stability_analysis(params);

%% 6. Parameter Sensitivity & Robustness Analysis (+/- 20% Sweep)
fprintf('\n[Step 6/7] Executing Parameter Sensitivity Sweep & Tornado Ranking...\n');
sensitivity_analysis(params);

%% 7. Obstacle Avoidance & Environmental Wind Disturbance Simulation
fprintf('\n[Step 7/7] Running 2D/3D Obstacle Avoidance & Wind Disturbance Trajectories...\n');
obstacle_simulation(params);

fprintf('\n*****************************************************************\n');
fprintf('* ALL SIMULATIONS, ANALYSES & FIGURES COMPLETED SUCCESSFULLY!   *\n');
fprintf('*****************************************************************\n');
