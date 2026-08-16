import numpy as np
import matplotlib.pyplot as plt

# Parameters from parameters.m
m_base = 1.5
T = 20.0
k_base = 0.5
g = 9.81
pitch_base = np.deg2rad(10.0)
roll = np.deg2rad(5.0)
t0 = 0.0
tf = 45.0
h = 0.1
t = np.arange(t0, tf + h, h)
N = len(t)

def analytical_solution(m, T, k, g, pitch, roll):
    Ax = (T/m) * np.sin(pitch) * np.cos(roll)
    Ay = -(T/m) * np.sin(roll)
    Az = (T/m) * np.cos(pitch) * np.cos(roll) - g
    
    vx0, vy0, vz0 = 0.0, 0.0, 0.0
    vx = (vx0 - Ax*m/k)*np.exp(-k*t/m) + Ax*m/k
    vy = (vy0 - Ay*m/k)*np.exp(-k*t/m) + Ay*m/k
    vz = (vz0 - Az*m/k)*np.exp(-k*t/m) + Az*m/k
    return vx, vy, vz

# Set MATLAB figure style
plt.style.use('default')
fig, axs = plt.subplots(3, 3, figsize=(14, 10), facecolor='white')
fig.suptitle('Sensitivity Analysis: Mass, Drag, and Pitch Variations', fontsize=15, fontweight='bold')

# 1. Mass variations: 1.0 kg vs 3.0 kg
vx_m1, vy_m1, vz_m1 = analytical_solution(1.0, T, k_base, g, pitch_base, roll)
vx_m3, vy_m3, vz_m3 = analytical_solution(3.0, T, k_base, g, pitch_base, roll)

# Subplot 1: Mass vx
axs[0, 0].plot(t, vx_m1, 'k', linewidth=2, label='1 kg')
axs[0, 0].plot(t, vx_m3, 'm--', linewidth=2, label='3 kg')
axs[0, 0].set_title('Mass: Forward Velocity (v_x)', fontweight='bold')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('v_x (m/s)')
axs[0, 0].legend()
axs[0, 0].grid(True)

# Subplot 2: Mass vy
axs[0, 1].plot(t, vy_m1, 'k', linewidth=2, label='1 kg')
axs[0, 1].plot(t, vy_m3, 'm--', linewidth=2, label='3 kg')
axs[0, 1].set_title('Mass: Lateral Velocity (v_y)', fontweight='bold')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('v_y (m/s)')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Subplot 3: Mass vz
axs[0, 2].plot(t, vz_m1, 'k', linewidth=2, label='1 kg')
axs[0, 2].plot(t, vz_m3, 'm--', linewidth=2, label='3 kg')
axs[0, 2].set_title('Mass: Vertical Velocity (v_z)', fontweight='bold')
axs[0, 2].set_xlabel('Time (s)')
axs[0, 2].set_ylabel('v_z (m/s)')
axs[0, 2].legend()
axs[0, 2].grid(True)

# 2. Drag variations: 0.5 vs 1.0
vx_d05, vy_d05, vz_d05 = analytical_solution(m_base, T, 0.5, g, pitch_base, roll)
vx_d10, vy_d10, vz_d10 = analytical_solution(m_base, T, 1.0, g, pitch_base, roll)

# Subplot 4: Drag vx
axs[1, 0].plot(t, vx_d05, 'k', linewidth=2, label='k = 0.5')
axs[1, 0].plot(t, vx_d10, 'r--', linewidth=2, label='k = 1.0')
axs[1, 0].set_title('Drag: Forward Velocity (v_x)', fontweight='bold')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('v_x (m/s)')
axs[1, 0].legend()
axs[1, 0].grid(True)

# Subplot 5: Drag vy
axs[1, 1].plot(t, vy_d05, 'k', linewidth=2, label='k = 0.5')
axs[1, 1].plot(t, vy_d10, 'r--', linewidth=2, label='k = 1.0')
axs[1, 1].set_title('Drag: Lateral Velocity (v_y)', fontweight='bold')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('v_y (m/s)')
axs[1, 1].legend()
axs[1, 1].grid(True)

# Subplot 6: Drag vz
axs[1, 2].plot(t, vz_d05, 'k', linewidth=2, label='k = 0.5')
axs[1, 2].plot(t, vz_d10, 'r--', linewidth=2, label='k = 1.0')
axs[1, 2].set_title('Drag: Vertical Velocity (v_z)', fontweight='bold')
axs[1, 2].set_xlabel('Time (s)')
axs[1, 2].set_ylabel('v_z (m/s)')
axs[1, 2].legend()
axs[1, 2].grid(True)

# 3. Pitch variations: 10 deg vs 25 deg
vx_p10, vy_p10, vz_p10 = analytical_solution(m_base, T, k_base, g, np.deg2rad(10.0), roll)
vx_p25, vy_p25, vz_p25 = analytical_solution(m_base, T, k_base, g, np.deg2rad(25.0), roll)

# Subplot 7: Pitch vx
axs[2, 0].plot(t, vx_p10, 'k', linewidth=2, label='10°')
axs[2, 0].plot(t, vx_p25, 'b--', linewidth=2, label='25°')
axs[2, 0].set_title('Pitch: Forward Velocity (v_x)', fontweight='bold')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].set_ylabel('v_x (m/s)')
axs[2, 0].legend()
axs[2, 0].grid(True)

# Subplot 8: Pitch vy
axs[2, 1].plot(t, vy_p10, 'k', linewidth=2, label='10°')
axs[2, 1].plot(t, vy_p25, 'b--', linewidth=2, label='25°')
axs[2, 1].set_title('Pitch: Lateral Velocity (v_y)', fontweight='bold')
axs[2, 1].set_xlabel('Time (s)')
axs[2, 1].set_ylabel('v_y (m/s)')
axs[2, 1].legend()
axs[2, 1].grid(True)

# Subplot 9: Pitch vz
axs[2, 2].plot(t, vz_p10, 'k', linewidth=2, label='10°')
axs[2, 2].plot(t, vz_p25, 'b--', linewidth=2, label='25°')
axs[2, 2].set_title('Pitch: Vertical Velocity (v_z)', fontweight='bold')
axs[2, 2].set_xlabel('Time (s)')
axs[2, 2].set_ylabel('v_z (m/s)')
axs[2, 2].legend()
axs[2, 2].grid(True)

plt.tight_layout()
plt.savefig('C:/Users/XPS/OneDrive - Ashesi University/Desktop/Differential/Drone Project/sensitivity_analysis.png', dpi=200)
plt.savefig('C:/Users/XPS/Downloads/differential/sensitivity_analysis.png', dpi=200)
print('Generated sensitivity_analysis.png matching MATLAB sensitivity_analysis.m successfully!')
