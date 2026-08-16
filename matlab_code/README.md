# KEMS UAV Autonomous Drone Simulation Suite (MATLAB)

This directory contains the modular MATLAB simulation suite for modeling, simulating, and analyzing the 6-DOF equations of motion for an autonomous agricultural quadrotor UAV with LiDAR obstacle avoidance.

---

## 🚀 Quick Start
1. Open MATLAB (R2020a or later).
2. Set this directory (`matlab_code/`) as your current working folder.
3. Run the master script:
   ```matlab
   main
   ```
4. The script will execute all simulations, run analytical benchmarks, generate error and stability plots, test parameter sensitivity, simulate obstacle avoidance, and launch the 3D rotor flight animation.

---

## 📂 File Overview

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

---

## 📐 Mathematical Model

The quadrotor translational equations of motion in the inertial frame are given by:

$$m \ddot{x} = (\cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi) U_1 - k_d \dot{x} + F_{w,x}$$

$$m \ddot{y} = (\cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi) U_1 - k_d \dot{y} + F_{w,y}$$

$$m \ddot{z} = (\cos\phi \cos\theta) U_1 - m g - k_d \dot{z}$$

Where:
* $U_1 = \sum f_i = b \sum \omega_i^2$ is the total motor thrust.
* $m = 1.888\text{ kg}$ is total quadrotor mass.
* $k_d = 0.45\text{ N}\cdot\text{s/m}$ is translational air resistance drag.
* $\phi, \theta, \psi$ are Roll, Pitch, and Yaw angles respectively.
* $g = 9.81\text{ m/s}^2$ is gravitational acceleration.

---

## 🔬 Solvers Comparison Summary

1. **Analytical Benchmark:** Exact closed-form formula used to evaluate numerical truncation errors.
2. **Euler's Method ($O(h)$):** Simplest implementation; conditionally stable for $h \le 0.1\text{ s}$, diverges for $h \ge 0.5\text{ s}$.
3. **Classical RK4 ($O(h^4)$):** High accuracy with fixed step sizes ($h = 0.02\text{ s}$), maintaining truncation errors below $10^{-4}\text{ m}$.
4. **Adaptive ode45:** Automatically adjusts time steps (taking small steps during obstacle turns and large steps during steady flight), providing optimal efficiency and precision.
