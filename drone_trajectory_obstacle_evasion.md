<!-- Page 1 -->

ASHESI UNIVERSITY 
CE122: APPLIED PROGRAMMING FOR ENGINEERS 
FINAL PROJECT: SUBMISSION THREE (WEEK 10) 
DRONE TRAJECTORY WITH OBSTACLE EVASION & SENSITIVITY ANALYSIS 
Faculty: Patrick Dwomfuor 
Group 5: 
Maureen Sedinam Agyei: 71682028 
Keziah Wilhelmina Naa Amerley Hammond: 11672028 
Elsa Ewuradwoa Hagan: 26602028 
Serwaa Abena Agyapong: 53612028

---

<!-- Page 2 -->

Contents 
tab ................................................................................................................... Error! Bookmark not defined. 
CHAPTER 1: INTRODUCTION ................................................................................................................. 3 
1.1 Background and Motivation................................................................................................................ 3 
1.2 Objectives ........................................................................................................................................... 4 
CHAPTER 2: SYSTEM DYNAMICS AND EQUATIONS OF MOTION ................................................. 5 
2.1 The Baseline State Vector and Parameters ......................................................................................... 5 
2.2 Continuous Flight Dynamics .............................................................................................................. 6 
CHAPTER 3: OBSTACLE EVASION DYNAMICS .................................................................................. 6 
3.1 Evasion Maneuver Algorithm ............................................................................................................. 6 
CHAPTER 4: COMPLETE SIMULATION RESULTS .............................................................................. 7 
4.1 Numerical Integration of the Avoidance Path ..................................................................................... 7 
4.2 Final State Comparison ....................................................................................................................... 7 
CHAPTER 5: STABILITY ANALYSIS ...................................................................................................... 8 
5.1 Euler Method Step Size Evaluation .................................................................................................... 8 
CHAPTER 6: PARAMETER SENSITIVITY ANALYSIS ......................................................................... 9 
6.1 Effect of Varying Mass (1.0 kg vs 3.0 kg) .......................................................................................... 9 
6.2 Effect of Varying Drag Coefficient (k=0.5 vs k=1.0) ......................................................................... 9 
6.3 Effect of Varying Pitch Angle (10° vs 25°) ...................................................................................... 10 
CHAPTER 7: ADVANCED VISUALIZATION & ANIMATION ........................................................... 11 
7.1 3D Solver Comparison Plot .............................................................................................................. 11 
7.2 Drone Flight Animation .................................................................................................................... 12 
CHAPTER 8: ENGINEERING INTERPRETATION & CONCLUSION................................................. 12 
8.1 Synthesis of Numerical Findings .................................................................................................. 12 
8.2 Engineering Implication .................................................................................................................... 13 
8.2 Conclusion ........................................................................................................................................ 13

---

<!-- Page 3 -->

CHAPTER 1: INTRODUCTION 
1.1 Background and Motivation 
The deployment of Uncrewed Aerial Vehicles (UAVs), commonly referred to as drones, has 
rapidly expanded across numerous engineering and commercial sectors, ranging from automated 
logistics and surveillance to agricultural monitoring and search-and-rescue operations. To safely 
and effectively navigate these complex, real-world airspaces, a UAV's autonomous control system 
must be built upon a highly accurate mathematical foundation. It is not enough for a quadcopter to 
merely understand its own internal flight dynamics; it must also possess the computational 
capability to process environmental feedback and react to sudden spatial hazards without losing 
stability. Mathematical modeling and numerical simulation serve as the critical first steps in 
designing these autonomous flight controllers, allowing engineers to safely test hardware 
configurations and evasive algorithms before deploying expensive physical prototypes. 
 
In Submission 2 of this project, a foundational dynamic model was successfully established to 
evaluate the three-dimensional translational motion of a quadcopter over a continuous 45-second 
flight. That phase validated the baseline system dynamics by solving a system of six first-order 
ordinary differential equations (ODEs) using three distinct numerical integration techniques: the 
first-order Euler Method, the Fourth-Order Runge-Kutta (RK4) Method, and MATLAB's 
dynamically adaptive ode45 solver.  
That analysis successfully demonstrated that the numerical approximations accurately captured 
the quadcopter's steady-state flight dynamics under the continuous influence of a 20 N thrust force,

---

<!-- Page 4 -->

a 1.5 kg mass, gravitational acceleration, and a 0.5 N.s/m aerodynamic drag coefficient. However, 
while that model accurately predicted uninterrupted linear flight paths, it represented an idealized 
environment. Real-world UAV operations require dynamic, autonomous course-correction 
capabilities to avoid collisions, necessitating a more advanced simulation model. 
 
1.2 Objectives  
This phase of our project represents the final optimisation of our quadcopter engineering 
simulation. The objectives of this project are built directly on the previously developed static 
trajectory models. This phase transitions the program into a dynamic, environment-aware 
simulation. The objectives are: 
• Obstacle Evasion Implementation: A time-triggered algorithm to simulate an 
autonomous obstacle avoidance manoeuvre, steering the drone safely around a 
programmed coordinate.  
• Numerical Solver Validation: Evaluating the Euler, RK4, and ode45 numerical 
methods to ensure they remain stable and accurate when processing sudden, non-
continuous state changes.  
• Parameter Sensitivity Analysis: Isolating key physical constraints (specifically UAV 
mass, aerodynamic drag coefficients, and pitch angles) to quantify how these variables 
impact dynamic responsiveness.

---

<!-- Page 5 -->

• Advanced Visualisation: Upgrading the simulation outputs with advanced 3D plotting 
and dynamic flight animations to visually verify the mathematical accuracy of the 
computed trajectories.  
• Engineering Interpretation: Providing a critical analysis of the simulation results to 
translate the mathematical data into engineering insights for future quadcopter design 
and autonomous control system optimisation. 
 
CHAPTER 2: SYSTEM DYNAMICS AND EQUATIONS OF 
MOTION 
 
2.1 The Baseline State Vector and Parameters 
 
The drone's state is defined by a 6-element vector containing its position (𝑥, 𝑦, 𝑧) and velocity 
components (𝑇𝑦𝑝𝑒 𝑒𝑞𝑢𝑎𝑡𝑖𝑜𝑛 ℎ𝑒𝑟𝑒.). The standard flight environment is governed by the following 
baseline parameters: 
• Mass (m): 1.5 kg  
• Total Thrust (T): 20 N  
• Linear Drag Coefficient (k): 0.5 N.s/m  
• Gravity (g): 9.81 m/s²  
• Pitch Angle (𝜃 ) : 10°

---

<!-- Page 6 -->

• Roll Angle (𝜑): 5° 
2.2 Continuous Flight Dynamics 
Before the evasion maneuver is triggered, the rates of change for the drone's velocities are 
formulated as ODEs integrating thrust, drag, and gravity 
 
CHAPTER 3: OBSTACLE EVASION DYNAMICS 
3.1 Evasion Maneuver Algorithm 
To evaluate the drone's ability to correct its flight path and simulate real-world autonomous 
navigation, a static obstacle is located at spatial coordinates (150, 20, 50).

---

<!-- Page 7 -->

Between 𝑡  =  20  seconds and 𝑡  =  25  seconds, an active evasion acceleration 𝑎𝑒𝑣𝑎𝑑𝑒 of 5 𝑚𝑠−2 
is applied. Outside of this precise 5-second window, 𝑎𝑒𝑣𝑎𝑑𝑒  =  0.  
Fill 
We use a piecewise logic that forces the quadcopter to abruptly move sideways during the specified 
time frame, effectively steering it around the obstacle before allowing it to restabilize. 
 
CHAPTER 4: COMPLETE SIMULATION RESULTS 
 
4.1 Numerical Integration of the Avoidance Path 
The modified trajectory was computed using the Euler Method, Fourth-Order Runge-Kutta (RK4), 
and ode45 solvers over the 45-second timeline with a high-quality baseline step size of ℎ  =  0.1  
seconds. 
 
4.2 Final State Comparison 
The numerical outputs at the end of the simulation (𝑡  =  45𝑠 ) confirm the impact of the evasion 
maneuver:  
• 
Euler Final State: 𝑥  =  290.6188𝑚 , 𝑦=   −69.9651𝑚 , 𝑧  =  412.1212𝑚  
• 
RK4 Final State: 𝑥  =  290.6188𝑚 , 𝑦  =   −70.9685𝑚 , 𝑧  =  412.1212𝑚   
• 
ode45 Final State: 𝑥  =  290.6188𝑚 , 𝑦=   −71.6348𝑚 , 𝑧  =  412.1212𝑚

---

<!-- Page 8 -->

While the x and z positions remain identical across solvers, slight variations exist in the y position. 
Higher-order solvers (RK4 and ode45) evaluate intermediate slopes, capturing the sudden 5-
second lateral thrust with greater mathematical precision than the first-order Euler approximation. 
 
CHAPTER 5: STABILITY ANALYSIS 
5.1 Euler Method Step Size Evaluation 
To evaluate the stability of the numerical approximations, the Euler Method was executed using 
three distinct step sizes: ℎ  =  0.1,  ℎ  =  0.5 𝑎𝑛𝑑 ℎ  =  1.0  seconds.

---

<!-- Page 9 -->

When a step size of ℎ  =  0.1  seconds was utilized, the numerical error remained highly stable and 
exceptionally small throughout the 45-second simulation. However, as the step size increased to 
ℎ  =  0.5  (growing error) and 1.0  (highly unstable), the absolute error in the forward velocity 𝑣𝑥 
spiked significantly. This demonstrates the core concept that first-order integration methods 
require high sampling frequencies to mitigate truncation errors. 
 
CHAPTER 6: PARAMETER SENSITIVITY ANALYSIS 
 
We evaluate the physical constraints of the drone. By isolating variables, the system's dynamic 
responsiveness was tested across three key parameters 
 
6.1 Effect of Varying Mass (1.0 kg vs 3.0 kg) 
The simulation compared a lightweight configuration (1.0 kg) against a heavy payload 
configuration (3.0 kg). The 3.0 kg drone exhibited a significantly slower rate of acceleration across 
all three velocity axes, taking much longer to reach terminal velocity. This aligns with the core 
concept of inertia. A larger mass resists changes in motion. With a constant thrust of 20 N, a 
heavier quadcopter requires more time to overcome drag and gravity. 
 
6.2 Effect of Varying Drag Coefficient (k=0.5 vs k=1.0) 
The aerodynamic resistance was doubled from the baseline 0.5 N.s/m to 1.0 N.s/m. Increasing the 
drag coefficient drastically reduced the final steady-state velocity in the forward, lateral, and

---

<!-- Page 10 -->

vertical directions. Aerodynamic drag acts as a direct dampening force. Higher air resistance forces 
the quadcopter to reach equilibrium (where thrust equals drag) at a much lower maximum speed, 
effectively capping its flight capabilities. 
6.3 Effect of Varying Pitch Angle (10° vs 25°) 
The forward tilt of the quadcopter was increased from 10° to 25°. The 25° pitch resulted in a 
massive increase in forward velocity (𝑣𝑥), while simultaneously causing a slight drop in vertical 
velocity (𝑣𝑧). Lateral velocity (𝑣𝑦) remained largely unaffected. Pitch angle determines thrust 
vectoring. By tilting further forward, a much larger percentage of the total 20 N thrust is directed 
along the horizontal axis, sacrificing vertical lift for forward momentum.

---

<!-- Page 11 -->

CHAPTER 7: ADVANCED VISUALIZATION & ANIMATION 
 
7.1 3D Solver Comparison Plot 
To map the drone's spatial progression, MATLAB's plot3 function was utilized to map the 
trajectories computed by the Euler, RK4, and ode45 solvers onto a three-dimensional grid. The 
visualization accurately places the static obstacle marker in 3D space, confirming visually that the 
5-second lateral thrust maneuver successfully altered the trajectory to avoid collision.

---

<!-- Page 12 -->

7.2 Drone Flight Animation 
A dynamic animation script was integrated to simulate the real-time flight of the UAV. The 
quadcopter is graphically rendered, updating its (x, y) coordinates at every iteration of the 0.1 time 
step. Please find video in file.  
 
CHAPTER 8: ENGINEERING INTERPRETATION & CONCLUSION 
8.1 Synthesis of Numerical Findings 
This final simulation successfully addressed the complexities of three-dimensional autonomous 
drone flight. By integrating a conditional, time-triggered lateral acceleration into the established 
equations of motion, the model effectively demonstrated an autonomous obstacle evasion 
maneuver.  
The numerical integration of this dynamic path yielded critical insights into solver performance. 
The final state arrays at t = 45 seconds revealed that while all solvers tracked the forward ($x$) 
and vertical (z) positions identically, the lateral (y) position varied slightly depending on the 
integration method used. The first-order Euler method calculated a final y-position of -69.9651m, 
whereas the higher-order RK4 and ode45 solvers calculated -70.9685m and -71.6348m, 
respectively. This discrepancy highlights the limitations of the Euler method in capturing sudden, 
piecewise accelerations. Because RK4 and ode45 evaluate multiple intermediate slopes within a 
single time step, they respond to the sudden activation and deactivation of the evasive thrust with 
significantly higher mathematical precision.

---

<!-- Page 13 -->

Furthermore, the stability analysis reinforced the necessity of high sampling rates when utilizing 
explicit numerical solvers. The Euler method remained highly stable only when the time step was 
minimized to h = 0.1. As the step size increased to 0.5 and 1.0, truncation errors accumulated 
rapidly, causing severe instability and rendering the simulated trajectory inaccurate. 
 
8.2 Engineering Implication 
The final optimized MATLAB simulation successfully addresses the complexities of 3D drone 
flight. The parameter sensitivity analysis confirms that UAV design requires careful balancing: 
higher payload masses degrade acceleration, while aerodynamic optimization (lowering drag) is 
critical for achieving high flight speeds. Furthermore, dynamic thrust vectoring (pitch control) is 
a highly effective method for regulating forward speed without altering total motor output. 
 
8.2 Conclusion 
Submission 3 successfully achieves all final project requirements. By implementing a time-
triggered lateral acceleration, the system achieved autonomous obstacle evasion. The Euler, RK4, 
and ode45 numerical solvers reliably processed this sudden piecewise state change, and the 
resulting advanced visualizations proved the maneuver's success. Ultimately, this optimized 
program serves as a robust foundational tool for modeling autonomous UAV navigation, predictive 
trajectory tracking, and physical parameter testing.

---

<!-- Page 14 -->

APPENDIX
