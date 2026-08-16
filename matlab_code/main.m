%% Drone Simulation
% AP Group 5
% Submission 3

clear;
clc;
close all;

%% 1. Load Parameters
% Loads physical parameters and initial conditions
params = parameters();

%% 2. Calculate Trajectories
% Run all four methods for the standard trajectory
[tAna, YAna]     = analytical_solution(params);
[tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics, params);
[tRK4, YRK4]     = rk4_solver(@drone_trajectory_dynamics, params);
[tODE45, YODE45] = ode45_solver(@drone_trajectory_dynamics, params);

% Display message from original main.m
disp('Euler solver completed successfully.');
disp('Final State Vector (Euler):');
disp(YEuler(:,end));

%% 3. Visualizations & Error Analysis
% Generates velocity comparisons and 3D trajectory
plot_results(tAna, YAna, YEuler, YRK4, YODE45);

% Compares numerical methods against analytical solution
error_analysis(tAna, YAna, YEuler, YRK4, YODE45);

%% 4. System Analyses
% Investigates Euler step sizes
stability_analysis(params);

% Investigates mass, drag, and pitch
sensitivity_analysis(params);

%% 5. Obstacle Avoidance & Animation
% Generates the obstacle avoidance plot
obstacle_simulation(params);

% Generate the specific trajectory to animate the drone
[~, YObstacle] = euler_solver(@drone_obstacle_dynamics, params);

% Animate the flight path
animate_drone(YObstacle, params);

%% 6. Export Figures (Optional)
% Uncomment the line below if you want to automatically save all generated plots
% export_results('Simulation_Figures');

disp('All simulations and analyses complete!');