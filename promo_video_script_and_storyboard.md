# KEMS UAV Autonomous Navigation Project — Official Promo Video Script & Storyboard

**Project Title:** Obstacle Avoidance-Based Autonomous Navigation of a Quadrotor System in Precision Agriculture  
**Courses:** MATH221 (Differential Equations & Numerical Methods) & CE122 (Applied Programming)  
**Institution:** Ashesi University  
**Target Video Duration:** 3 Minutes (180 Seconds)  
**Format:** High-energy academic & visual pitch combining live website screencasts, MATLAB simulation animations, equation callouts, and real-world drone footage.

---

## 🎬 Master Production Overview

| Scene | Timecode | Topic / Section | Visual Assets & B-Roll | Primary Speaker |
|:---:|:---:|---|---|:---:|
| **1** | 0:00 – 0:28 | **The Hook:** Livestock Challenges on African Savannas | Aerial footage of Ghanaian savanna pastures / grazing herds | Speaker 1 (Introduction) |
| **2** | 0:28 – 0:58 | **The Math:** 6-DOF Newton-Euler Differential Equations | Website Equation view + 3D Quadrotor tilt diagram | Speaker 2 (Model Derivation) |
| **3** | 0:58 – 1:30 | **Control & Tuning:** ITAE vs. ISE Controller Dynamics | Side-by-side Figure 11 & 12 replication curves | Speaker 3 (Study Results) |
| **4** | 1:30 – 2:05 | **Our Engineering Modifications:** Wind, Drag & Solvers | Live HTML5 Flight Simulator + White Academic MATLAB Charts | Speaker 4 (Modifications) |
| **5** | 2:05 – 2:38 | **Applied Programming:** Modular MATLAB Architecture | MATLAB R2024b code suite + Error convergence log-log plots | Speaker 4 (Applied Programming) |
| **6** | 2:38 – 3:00 | **Conclusion & Future Horizons:** Autonomous Agritech | Swarm drone visual + Website URL callout | All Speakers (Closing) |

---

## 📜 Scene-by-Scene Script & Storyboard

```
[0:00 - 0:05]
[AUDIO CUE: Upbeat, modern electronic STEM background music begins softly at 20% volume]
[VISUAL: Cinematic aerial drone footage over the lush green rolling pastures of the Afram Plains at sunrise. Title card animates: "KEMS UAV Autonomous Navigation Project".]
```

### Scene 1: The Challenge in Precision Agriculture (0:00 – 0:28)
* **Speaker 1 (Introduction):**  
  *"Across large-scale livestock pastures in Ghana and Sub-Saharan Africa, tracking grazing livestock across hundreds of hectares is one of the most grueling challenges in agriculture. Over 60% of animal losses occur in unmonitored blind spots, dense acacia scrub, and rocky gullies.*  
  *Manual herding under scorching heat is slow and risky—but what if an autonomous aerial robot could scout the pasture, monitor grazing herds, and steer around trees and fences entirely on its own?*  
  *Welcome to the KEMS UAV Autonomous Navigation Project, where we turn ordinary differential equations into intelligent autonomous flight."*

```
[0:28 - 0:32]
[VISUAL: Smooth transition to the project website (https://differential-kems.vercel.app/). Animated camera zooms into the "Understanding the Model" section showing the quadcopter free-body diagram with 4 spinning rotors.]
```

### Scene 2: The Differential Equations of Motion (0:28 – 0:58)
* **Speaker 2 (Mathematical Modeling):**  
  *"To achieve autonomous flight, we model the quadcopter as a 6-degree-of-freedom rigid body governed by Newton’s Second Law and Euler’s rotational dynamics.*  
  *A quadrotor is under-actuated: it has six outputs—position $X, Y, Z$ and Euler angles Roll $\phi$, Pitch $\theta$, and Yaw $\psi$—but only four motor inputs.*  
  *To fly forward in the $X$-direction, the drone doesn't have a horizontal propeller. Instead, back motor 3 spins faster than front motor 1, tilting the pitch angle $\theta$ forward. This tilts the total upward thrust $U_1$, redirecting part of that vertical lift horizontally to drive acceleration.*  
  *Meanwhile, an onboard 2D LiDAR rangefinder continuously scans for obstacles within a 1.0-meter safety radius."*

```
[0:58 - 1:02]
[VISUAL: Screen displays Figure Replication 1 & 2 side-by-side with original paper figures and our replicated MATLAB step-response graphs. Red highlights flash on the 14.30% overshoot peak vs the 2.65% ITAE curve.]
```

### Scene 3: Controller Tuning & Published Study Results (0:58 – 1:30)
* **Speaker 3 (Control & Results Discussion):**  
  *"In our study of Alanezi et al. (MDPI Drones 2022), the authors tuned dual PID feedback controllers using Genetic Algorithms under two cost criteria: Integral Square Error (ISE) and Integral Time Absolute Error (ITAE).*  
  *Our replicated simulations reveal why ITAE is vastly superior: ISE squares early errors, causing the motors to surge violently, resulting in a dangerous 14.30% altitude overshoot.*  
  *In contrast, ITAE weights error by time, yielding an optimal damping ratio $\zeta = 0.8158$ with only 2.65% overshoot and a 1-millimeter steady-state error—ensuring the drone never crashes into livestock or tree canopies during vertical ascent."*

```
[1:30 - 1:35]
[VISUAL: Transition to the live interactive HTML5 Flight Simulator on the website. Cursor drags the "Lateral Wind Disturbance" slider to +0.15N and toggles "3 Obstacles", showing the drone dynamically banking and evading in real time.]
```

### Scene 4: Our Modifications & Environmental Robustness (1:30 – 2:05)
* **Speaker 4 (Model Modifications & Numerical Solvers):**  
  *"While the published paper assumed calm indoor air and static obstacles, real-world African farms face harsh environmental disturbances.*  
  *We expanded the equations of motion by introducing aerodynamic body drag ($-k_d v$), variable payload masses ($1.4\text{ kg}$ to $2.5\text{ kg}$ for multi-spectral crop cameras), and continuous crosswind gusts ($F_{w,x}(t)$).*  
  *On our interactive web testbed, you can dynamically adjust wind force, mass, and sensor latency, observing how the Bat Algorithm evasion vectors maintain safe 1.0-meter clearance."*

```
[2:05 - 2:10]
[VISUAL: Screen switches to the MATLAB R2024b IDE. We run `main.m`. Authentic MATLAB figure windows pop up showing: (1) Altitude step response, (2) Error convergence log-log plot, (3) Numerical stability breakdown.]
```

### Scene 5: Applied Programming & Multi-Solver Analysis (2:05 – 2:38)
* **Speaker 4 (Applied Programming & Error Analysis):**  
  *"For our Applied Programming codebase in CE122, we built a modular 14-script MATLAB suite.*  
  *We benchmarked the numerical methods taught in class: MATLAB's adaptive `ode45`, Classical fixed-step `RK4`, and `Forward Euler` against our exact analytical closed-form solution.*  
  *`ode45` proved to be the champion solver: its adaptive step size takes micro-steps around tight obstacle turns and large steps in straight hover, achieving $0.04\text{ mm}$ precision in just $1.84\text{ ms}$.*  
  *Our log-log convergence analysis proves RK4 achieves fourth-order accuracy $O(h^4)$, while Forward Euler is only conditionally stable ($h < 0.10\text{s}$) and completely blows up if step size exceeds the critical threshold."*

```
[2:38 - 2:42]
[AUDIO CUE: Music swells to a grand, inspiring crescendo.]
[VISUAL: Dynamic montage of the interactive website, the creative science infographic in motion, and 3D MATLAB flight trajectories. Final slide displays project credits, Ashesi University emblem, and live website URL.]
```

### Scene 6: Agritech Impact & Conclusion (2:38 – 3:00)
* **All Speakers:**  
  *"By bridging theoretical ordinary differential equations, control theory, and numerical algorithms, the KEMS UAV project demonstrates how autonomous robotics can revolutionize precision agriculture across Ghana and beyond.*  
  *Explore our interactive simulator, inspect our complete open-source MATLAB codebase, and test the equations yourself at **differential-kems.vercel.app**.*  
  *Thank you!"*

---

## 🛠️ Recording Tips & Production Guidelines for the Team

1. **Screen Recording Tools:** Use OBS Studio or QuickTime at 1080p 60fps to capture smooth 60fps flight simulator animations on `http://localhost:8000/` or `https://differential-kems.vercel.app/`.
2. **Audio Quality:** Use a USB microphone or headset with noise suppression. Speak clearly with confident, enthusiastic pacing.
3. **MATLAB Live Run:** Capture a quick screen recording of running `main.m` in MATLAB R2024b to show the figures popping up live!
