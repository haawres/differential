%% KEMS UAV: AUTONOMOUS QUADROTOR NAVIGATION & SIMULATION REPORT
% *Differential Equations & Numerical Methods Project*
%
% *Team Members & Contributions:*
%
% * *Keziah Wilhemina Hammond* - Applied Programming, Mathematical Modelling and Result Interpretation
% * *Elsa AwuraAdwoa Hagan* - Applied Programming, Obstacle Avoidance Dynamics
% * *Maureen Sedinam Agyei* - Numerical Solvers & Model Modifications
% * *Serwaa Abena Agyapong* - Parameter Sensitivity Analysis & Stability Evaluation
%
%% 1. SYSTEM PHYSICAL PARAMETERS
% Loads the physical constants and initial state vector.
type('parameters.m')
params = parameters();

%% 2. EXACT ANALYTICAL CLOSED-FORM SOLUTION
% Computes the exact ground-truth velocity and altitude profiles.
type('analytical_solution.m')
[tAna, YAna] = analytical_solution(params);

%% 3. 6-DOF EQUATIONS OF MOTION (DYNAMIC ODEs)
% Evaluates state derivatives for translational positions and rotational angles.
type('drone_trajectory_dynamics.m')

%% 4. NUMERICAL INTEGRATION SOLVERS
% Implementation of Euler's Method, Classical Runge-Kutta (RK4), and adaptive ode45.
%
% *A. Euler's Method (1st-Order):*
type('euler_solver.m')
[tEuler, YEuler] = euler_solver(@drone_trajectory_dynamics, params);

% *B. Classical 4th-Order Runge-Kutta (RK4):*
type('rk4_solver.m')
[tRK4, YRK4] = rk4_solver(@drone_trajectory_dynamics, params);

% *C. Adaptive ode45 (Dormand-Prince):*
type('ode45_solver.m')
[tODE45, YODE45] = ode45_solver(@drone_trajectory_dynamics, params);

%% 5. SIMULATION RESULTS: 3D TRAJECTORY & VELOCITIES
% Compares the 3D flight paths and translational velocities across all methods.
type('plot_results.m')
plot_results(tAna, YAna, YEuler, YRK4, YODE45);

%% 6. NUMERICAL ERROR CONVERGENCE ANALYSIS
% Benchmarks absolute truncation errors vs the exact analytical benchmark.
type('error_analysis.m')
error_analysis(tAna, YAna, YEuler, YRK4, YODE45);

%% 7. EULER STEP-SIZE NUMERICAL STABILITY ANALYSIS
% Evaluates conditional stability for step sizes h = 0.1, 0.5, 1.0 s.
type('stability_analysis.m')
stability_analysis(params);

%% 8. SYSTEM PARAMETER SENSITIVITY & ROBUSTNESS STUDY
% Investigates variations in Drone Mass (+-20%), Drag Coefficient, and Pitch Angle.
type('sensitivity_analysis.m')
sensitivity_analysis(params);

%% 9. 2D LIDAR OBSTACLE AVOIDANCE DYNAMICS
% Evaluates repulsive steering forces to smoothly steer around obstacles.
type('drone_obstacle_dynamics.m')
type('obstacle_simulation.m')
obstacle_simulation(params);

%% 10. 3D FLIGHT VISUALIZATION & ROTOR ANIMATION
type('animate_drone.m')
[~, YObstacle] = euler_solver(@drone_obstacle_dynamics, params);
animate_drone(YObstacle, params);

%% 11. PUBLISHED PAPER FIGURE REPLICATIONS (Alanezi et al., Drones 2022)
%
% *Figure 1: Altitude Step Response (ITAE vs ISE)*
type('replicate_fig1_altitude.m')
replicate_fig1_altitude();

% *Figure 2: Attitude Tilt Angles (Roll, Pitch, Yaw)*
type('replicate_fig2_attitude.m')
replicate_fig2_attitude();

% *Figure 3: Single Obstacle Avoidance Trajectory*
type('replicate_fig3_single_obstacle.m')
replicate_fig3_single_obstacle();

% *Figure 4: Multiple Obstacle Navigation Trajectory*
type('replicate_fig4_multi_obstacle.m')
replicate_fig4_multi_obstacle();

disp('Complete project report compiled successfully!');
