# Drone Simulation & Figure Replication (MATLAB)

This folder contains the MATLAB simulation scripts for the drone flight trajectory, obstacle avoidance, sensitivity analysis, and paper figure replications.

---

## 🚀 How to Run

### 1. Run Complete Simulation:
```matlab
main
```
Runs the full simulation pipeline (parameters, analytical benchmark, numerical solvers, error analysis, stability, sensitivity, obstacle avoidance, and animation).

### 2. Run Figure Replications:
```matlab
figure_replication
```
Generates the four figure replication plots shown on the website (Altitude step response, Attitude angles, Single obstacle avoidance, and Multiple obstacle navigation).

### 3. Publish to PDF:
Open `final_codes.m` in MATLAB and click **Publish** in the Editor ribbon.

---

## 📂 File Directory

| File | Description |
| :--- | :--- |
| `final_codes.m` | Complete compiled script containing all simulation and replication codes ready for single-file execution or publishing. |
| `main.m` | Master simulation script. |
| `figure_replication.m` | Replicates the 4 paper figures from the website (Figures 1, 2, 3, and 4). |
| `parameters.m` | Loads drone physical constants and initial conditions. |
| `analytical_solution.m` | Computes the exact closed-form analytical solution. |
| `drone_trajectory_dynamics.m` | 6-DOF drone state derivatives. |
| `drone_obstacle_dynamics.m` | State derivatives with lateral obstacle avoidance. |
| `euler_solver.m` | Euler Method numerical integrator. |
| `rk4_solver.m` | Fourth-Order Runge-Kutta integrator. |
| `ode45_solver.m` | MATLAB adaptive ode45 solver. |
| `plot_results.m` | Plots 3D flight trajectory and translational velocities. |
| `error_analysis.m` | Compares numerical errors against the analytical benchmark. |
| `stability_analysis.m` | Evaluates Euler step size stability ($h = 0.1, 0.5, 1.0\text{ s}$). |
| `sensitivity_analysis.m` | 3x3 subplot grid for Mass, Drag, and Pitch variations. |
| `obstacle_simulation.m` | Plots 2D obstacle avoidance trajectory. |
| `animate_drone.m` | 2D/3D flight animation of drone avoiding an obstacle. |
