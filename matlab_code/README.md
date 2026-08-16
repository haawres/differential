# KEMS UAV Autonomous Drone Simulation Suite (MATLAB)

This repository contains the complete, modular MATLAB simulation suite for modeling, simulating, and analyzing the 6-DOF equations of motion for an autonomous agricultural quadrotor UAV with LiDAR obstacle avoidance.

---

## 🚀 Quick Start

### 1. Run Complete Simulation & Analysis:
```matlab
main
```
Executes physical parameter loading, exact analytical solution, numerical solvers (Euler, RK4, ode45), error analysis, stability testing, parameter sensitivity sweeps, obstacle avoidance, and 3D flight animation.

### 2. Run Published Paper Figure Replications:
```matlab
replicate_paper_figures
```
Recreates all four figures published in *Alanezi et al., MDPI Drones 2022, 6, 288* (Altitude step response, Attitude angles, Single-obstacle evasion, and Multi-obstacle trajectory).

---

## 📂 File Directory & Modules

### Core Simulation Pipeline
| File | Type | Description |
| :--- | :--- | :--- |
| `main.m` | Script | **Master simulation runner**. Orchestrates parameter loading, solver execution, comparisons, and visualizations. |
| `parameters.m` | Function | Returns structure `params` containing physical constants ($m=1.888\text{ kg}$, $l=0.225\text{ m}$, $b, d, I_x, I_y, I_z$) and initial conditions. |
| `analytical_solution.m` | Function | Computes the closed-form exact solution for the linearized altitude second-order ODE and velocity equations. |
| `drone_trajectory_dynamics.m` | Function | Evaluates state derivatives $\dot{\mathbf{Y}} = f(t, \mathbf{Y})$ for 6-DOF translation and Euler rotation with linear aerodynamic drag. |
| `drone_obstacle_dynamics.m` | Function | Evaluates state derivatives with 2D LiDAR repulsive steering acceleration for obstacle avoidance. |
| `euler_solver.m` | Function | First-order fixed-step numerical integrator ($\mathbf{Y}_{n+1} = \mathbf{Y}_n + h \cdot f(t_n, \mathbf{Y}_n)$). |
| `rk4_solver.m` | Function | Classical fourth-order Runge-Kutta integrator evaluating four weighted slope estimates per time step. |
| `ode45_solver.m` | Function | Adaptive-step Runge-Kutta (Dormand-Prince pair) solver using MATLAB's built-in `ode45`. |
| `plot_results.m` | Function | Generates comparative plots of 3D trajectory and translational velocities ($v_x, v_y, v_z$). |
| `error_analysis.m` | Function | Computes absolute error $|v_{\text{num}} - v_{\text{ana}}|$ across time for each numerical solver. |
| `stability_analysis.m` | Function | Evaluates Euler method numerical stability across step sizes $h \in [0.1, 0.5, 1.0\text{ s}]$. |
| `sensitivity_analysis.m` | Function | Generates a 9-panel subplot grid testing system robustness under variations in Mass, Drag, and Pitch angle. |
| `obstacle_simulation.m` | Function | Simulates and plots the 2D path of the drone navigating smoothly around an obstacle to reach target coordinates. |
| `animate_drone.m` | Function | Renders a 3D visual animation of the quadrotor frame with rotating rotors along the flight path. |

### Published Paper Figure Replication Suite
| File | Type | Description |
| :--- | :--- | :--- |
| `replicate_paper_figures.m` | Script | **Master runner** that executes all four figure replication scripts sequentially. |
| `replicate_fig1_altitude.m` | Function | Replicates **Figure 11**: Altitude Step Response ($z = 5.0\text{ m}$) under ITAE (2.65% overshoot) vs ISE (14.30% overshoot) tuning. |
| `replicate_fig2_attitude.m` | Function | Replicates **Figure 12**: Roll ($\phi$), Pitch ($\theta$), and Yaw ($\psi$) attitude angle step responses comparing ITAE vs ISE. |
| `replicate_fig3_single_obstacle.m` | Function | Replicates **Figure 13**: Single-obstacle navigation trajectory around an obstacle at $(15, 20)$ with a 1.0 m safe clearance zone. |
| `replicate_fig4_multi_obstacle.m` | Function | Replicates **Figure 14**: Multi-obstacle navigation trajectory through three sequential obstacles to reach $(50, 50)$. |

---

## 📐 Mathematical Model

The quadrotor translational equations of motion in the inertial frame are given by:

$$m \ddot{x} = (\cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi) U_1 - k_d \dot{x} + F_{w,x}$$

$$m \ddot{y} = (\cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi) U_1 - k_d \dot{y} + F_{w,y}$$

$$m \ddot{z} = (\cos\phi \cos\theta) U_1 - m g - k_d \dot{z}$$

Where:
* $U_1 = \sum f_i = b \sum \omega_i^2$ is total motor thrust.
* $m = 1.888\text{ kg}$ is quadrotor mass.
* $k_d = 0.45\text{ N}\cdot\text{s/m}$ is translational air drag.
* $\phi, \theta, \psi$ are Roll, Pitch, and Yaw angles respectively.
* $g = 9.81\text{ m/s}^2$ is gravitational acceleration.
