# 🎬 2-Minute Promo Video Script & Production Storyboard

**Project Title:** Obstacle Avoidance-Based Autonomous Navigation of a Quadrotor System  
**Courses:** MATH221 (Differential Equations & Numerical Methods) & CE122 (Applied Programming)  
**Authors:** KEMS UAV Research Group — Ashesi University  
**Target Duration:** Exactly 120 Seconds (2 Minutes)  
**Music Track:** Uplifting, fast-paced electronic/ambient STEM soundtrack (e.g., synth pads + subtle driving tech beat)  

---

## ⏱️ Video Structure Overview

| Scene | Time Window | Title / Section Focus | Key Message |
|:---:|:---:|:---|:---|
| **1** | `00:00 - 00:20` | **The Hook & Problem Statement** | Livestock monitoring challenges & why UAV autonomous navigation matters |
| **2** | `00:20 - 00:45` | **The Mathematical Formulation** | 6-DOF Newton-Euler ODEs & Genetic Algorithm-optimized PID controllers |
| **3** | `00:45 - 01:10` | **Our Contributions & Modifications** | Wind disturbance injection, payload mass variations, non-linear body drag |
| **4** | `01:10 - 01:35` | **Numerical Methods & Error Analysis** | Solver benchmarking (`ode45` vs `ode23` vs `RK4` vs Euler) & convergence |
| **5** | `01:35 - 01:50` | **Interactive Flight Simulator & Results** | Real-time web simulator, ITAE vs ISE performance, 1.0m safety threshold |
| **6** | `01:50 - 02:00` | **Closing & Future Frontiers** | Multi-agent swarms, GPS-denied SLAM & team credits |

---

## 📽️ Scene-by-Scene Production Script

### 🎬 Scene 1: The Hook & Precision Agriculture (00:00 - 00:20)
* **Visual on Screen:**  
  * Aerial video footage / 3D render of agricultural pasture with grazing sheep.
  * Animated quadcopter drone swooping in smoothly over the landscape with glowing telemetry HUD overlays.
  * Graphic showing obstacles: trees, fence lines, and livestock.
* **On-Screen Text (Lower Third):**  
  `Autonomous Quadrotor Navigation | Livestock Surveillance & Tracking`
* **Voiceover Narration:**  
  > *"Managing expansive livestock pastures requires continuous aerial monitoring, counting, and health surveillance. But navigating a quadrotor through dynamic outdoor terrain full of natural obstacles demands more than human piloting—it demands rigorous mathematical modelling and autonomous control."*

---

### 🎬 Scene 2: The Differential Equations & Control System (00:20 - 00:45)
* **Visual on Screen:**  
  * Zoom-in on the creative infographic diagram showing the quadcopter's 6 degrees of freedom ($\phi, \theta, \psi, x, y, z$).
  * Dynamic equations fading in:
    $$\ddot{x} = \frac{U_1}{m}(\cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi)$$
    $$\ddot{z} = \frac{U_1}{m}(\cos\phi\cos\theta) - g$$
  * PID Controller block diagram animation showing error calculation and motor thrust mapping: $f_i = b \cdot \omega_i^2$.
* **On-Screen Text:**  
  `6-DOF Newton-Euler Dynamic Formulation | GA-Tuned ITAE vs. ISE PID Controllers`
* **Voiceover Narration:**  
  > *"We modelled the quadrotor as a six-degree-of-freedom under-actuated rigid body. Six output states are driven by four rotor inputs using the Newton-Euler formalism. Using Genetic Algorithms, we optimized dual PID feedback loops under both Integral of Square Error (ISE) and Integral of Time-weighted Absolute Error (ITAE) cost functions."*

---

### 🎬 Scene 3: Our Contributions & Real-World Modifications (00:45 - 01:10)
* **Visual on Screen:**  
  * Side-by-side simulation screen showing a drone flying in still air vs. drifting under a crosswind.
  * Animated wind force vector ($F_{\text{wind}}$) and aerodynamic quadratic drag force equation:
    $$\mathbf{F}_{\text{drag}} = -\frac{1}{2}\rho C_d A \|\mathbf{v}\|\mathbf{v}$$
  * Slider showing payload variation (1.5 kg to 2.5 kg camera rig) and its effect on motor thrust requirements.
* **On-Screen Text:**  
  `Our Contribution: Environmental Wind Disturbances + Dynamic Payload Mass Sensitivity`
* **Voiceover Narration:**  
  > *"The original paper assumed calm air and fixed mass. To bridge the gap to real-world deployment, our project introduced lateral crosswind disturbances, stochastic wind gusts, and non-linear aerodynamic body drag directly into the differential equations. We systematically investigated how variable camera payloads impact controller stability."*

---

### 🎬 Scene 4: Numerical Methods & Solver Benchmarking (01:10 - 01:35)
* **Visual on Screen:**  
  * MATLAB / Python benchmark plots: Log-log convergence curves showing Forward Euler $O(h)$ vs. Classical Runge-Kutta 4th Order $O(h^4)$.
  * Multi-solver comparison bar charts showing execution runtime and RMSE across `ode45`, `ode23`, `ode15s`, `ode113`, and RK4.
  * Analytical closed-form solution curve overlaid with numerical approximations.
* **On-Screen Text:**  
  `Applied Programming: Solver Convergence $O(h^4)$ | RMSE Error Benchmarking`
* **Voiceover Narration:**  
  > *"For Applied Programming, we solved the linearized system analytically and benchmarked six numerical integration schemes in MATLAB R2024b. While Forward Euler requires micro-steps to prevent numerical divergence, adaptive Dormand-Prince ode45 achieved sub-millimeter precision with over 90% fewer function evaluations."*

---

### 🎬 Scene 5: Interactive Simulator & Key Findings (01:35 - 01:50)
* **Visual on Screen:**  
  * Screen capture of our web-based interactive flight simulator running on `http://localhost:8000/`.
  * User dragging the lateral wind force slider and toggling between ITAE and ISE loops.
  * Demonstration of the 1.0m safety radius steering smoothly around obstacles vs 0.5m collision threshold.
* **On-Screen Text:**  
  `Web-Based Flight Simulator | Real-Time Telemetry & Bat Algorithm Avoidance`
* **Voiceover Narration:**  
  > *"Our interactive browser simulator proves that ITAE tuning provides superior damping with only 2.65% overshoot compared to 14.3% in ISE, while a 1.0-meter obstacle clearance threshold guarantees collision-free navigation even under severe sensor latency."*

---

### 🎬 Scene 6: Conclusion & Future Research Frontiers (01:50 - 02:00)
* **Visual on Screen:**  
  * Animated 3D drone flight animation with trail trace reaching the final waypoint.
  * Brief visual icons for Future Work: Swarm Collaboration, GPS-Denied Visual SLAM, and Energy-Optimal Trajectories.
  * Team credits card: Ashesi University — KEMS UAV Research Group.
* **On-Screen Text:**  
  `Diff'd Up UAV | MATH221 & CE122 Final Project | Ashesi University`
* **Voiceover Narration:**  
  > *"From mathematical derivation to real-time interactive simulation, our work lays the foundation for multi-agent swarm coordination and smart agricultural robotics. Explore our full research and flight simulator on our website. Thank you!"*

---

## 🎙️ Recording & Production Tips for Students

1. **Audio Recording:** Use a clear microphone (or headset mic), speak at a steady pace (~130 words per minute), and keep energy high and articulate.
2. **Screen Recording:** Capture the interactive simulator on full screen at 1080p, 60fps using OBS Studio or PowerPoint Screen Recorder.
3. **Editing Software:** Use CapCut, DaVinci Resolve, Premiere Pro, or Clipchamp to sync voiceover with on-screen equation callouts and MATLAB plots.
4. **Music Volume:** Keep background music ducked at -18dB when voiceover is speaking, and raise to -10dB during the opening and closing transitions.
