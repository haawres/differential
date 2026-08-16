%% Replicate Published Paper Figures (Alanezi et al., Drones 2022)
% KEMS UAV Autonomous Quadrotor Project
% Master runner script for all Figure Replications

clear;
clc;
close all;

disp('======================================================');
disp('   KEMS UAV: Paper Figure Replication Suite');
disp('======================================================');

%% 1. Replicate Figure 1 (Paper Fig 11): Altitude Step Response
disp('[1/4] Generating Figure 1: Altitude Step Response (ITAE vs ISE)...');
replicate_fig1_altitude();

%% 2. Replicate Figure 2 (Paper Fig 12): Attitude Step Response (Roll, Pitch, Yaw)
disp('[2/4] Generating Figure 2: Attitude Tilt Angles (Roll, Pitch, Yaw)...');
replicate_fig2_attitude();

%% 3. Replicate Figure 3 (Paper Fig 13): Single Obstacle Avoidance Path
disp('[3/4] Generating Figure 3: Single Obstacle Navigation Trajectory...');
replicate_fig3_single_obstacle();

%% 4. Replicate Figure 4 (Paper Fig 14): Multiple Obstacle Flight Path
disp('[4/4] Generating Figure 4: Multiple Obstacle Navigation Trajectory...');
replicate_fig4_multi_obstacle();

disp('======================================================');
disp('   All 4 Figures Replicated Successfully!');
disp('======================================================');
