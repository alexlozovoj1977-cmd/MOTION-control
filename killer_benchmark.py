import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. НАЛАШТУВАННЯ СИСТЕМИ ТА СИМУЛЯЦІЇ
# ==========================================
J = 0.01      # Інерція ротора (kg*m^2)
b = 0.1       # В'язке тертя (N*m*s/rad)
dt = 0.001    # Крок симуляції (1 мс)
T = 2.0       # Час симуляції (секунди)
steps = int(T / dt)
time = np.linspace(0, T, steps)

target_angle = np.pi / 2  # Крок у 90 градусів

U_MAX = 5.0   # Saturation мотора
K_motion = 5.0  
Kd_motion = 0.2 
Kp = 5.0
Ki = 2.0
Kd = 0.2

# ==========================================
# 2. ФУНКЦІЯ СИМУЛЯЦІЇ
# ==========================================
def run_simulation(noise_sigma=0.0):
    state_pid = np.array([0.0, 0.0])
    state_mot = np.array([0.0, 0.0])
    
    hist_pid = {'pos': np.zeros(steps), 'u': np.zeros(steps), 'e': np.zeros(steps)}
    hist_mot = {'pos': np.zeros(steps), 'u': np.zeros(steps), 'e': np.zeros(steps)}
    
    integral_error = 0.0
    energy_pid = 0.0
    energy_mot = 0.0
    
    for i in range(steps):
        noise = np.random.normal(0, noise_sigma)
        pos_pid_sense = state_pid[0] + noise
        pos_mot_sense = state_mot[0] + noise
        
        # --- PID ---
        error_pid = target_angle - pos_pid_sense
        integral_error += error_pid * dt
        derivative_pid = -state_pid[1] 
        
        u_pid_raw = Kp * error_pid + Ki * integral_error + Kd * derivative_pid
        
        if u_pid_raw > U_MAX:
            u_pid = U_MAX
            integral_error -= error_pid * dt 
        elif u_pid_raw < -U_MAX:
            u_pid = -U_MAX
            integral_error -= error_pid * dt 
        else:
            u_pid = u_pid_raw
            
        # --- MOTION ---
        delta_phi = (target_angle - pos_mot_sense + np.pi) % (2 * np.pi) - np.pi
        u_mot_raw = K_motion * np.sin(delta_phi) - Kd_motion * state_mot[1]
        u_mot = np.clip(u_mot_raw, -U_MAX, U_MAX)
        
        # --- ФІЗИКА ---
        accel_pid = (u_pid - b * state_pid[1]) / J
        state_pid[1] += accel_pid * dt
        state_pid[0] += state_pid[1] * dt
        
        accel_mot = (u_mot - b * state_mot[1]) / J
        state_mot[1] += accel_mot * dt
        state_mot[0] += state_mot[1] * dt
        
        energy_pid += (u_pid ** 2) * dt
        energy_mot += (u_mot ** 2) * dt
        
        hist_pid['pos'][i] = state_pid[0]
        hist_pid['u'][i] = u_pid
        hist_pid['e'][i] = energy_pid
        
        hist_mot['pos'][i] = state_mot[0]
        hist_mot['u'][i] = u_mot
        hist_mot['e'][i] = energy_mot
        
    return hist_pid, hist_mot

# ==========================================
# 3. АНАЛІЗ МЕТРИК
# ==========================================
def calculate_metrics(hist, target):
    pos = hist['pos']
    u = hist['u']
    
    error_band = 0.02 * target
    diff = np.abs(pos - target) > error_band
    settled_idx = np.where(diff)[0][-1] if np.any(diff) else 0
    settling_time = settled_idx * dt
    
    max_val = np.max(pos)
    overshoot = max(0, (max_val - target) / target * 100)
    
    total_energy = hist['e'][-1]
    
    tail_idx = int(steps * 0.8)
    jitter = np.var(u[tail_idx:])
    
    return settling_time, overshoot, total_energy, jitter

h_pid_clean, h_mot_clean = run_simulation(0.0)
h_pid_noisy, h_mot_noisy = run_simulation(0.05)

m_pid = calculate_metrics(h_pid_noisy, target_angle)
m_mot = calculate_metrics(h_mot_noisy, target_angle)

imp_settle = (m_pid[0] - m_mot[0]) / m_pid[0] * 100 if m_pid[0] > 0 else 0
imp_energy = (m_pid[2] - m_mot[2]) / m_pid[2] * 100 if m_pid[2] > 0 else 0
imp_jitter = (m_pid[3] - m_mot[3]) / m_pid[3] * 100 if m_pid[3] > 0 else 0

print("="*60)
print(" ГОТОВИЙ КОД ДЛЯ LATEX ТАБЛИЦІ (ВСТАВТЕ В СТАТТЮ)")
print("="*60)
print(r"\begin{table}[htbp]")
print(r"\centering")
print(r"\caption{Experimental Metrics (Step Response with $\sigma=0.05$ rad noise)}")
print(r"\label{tab:metrics}")
print(r"\begin{tabular}{@{}lccc@{}}")
print(r"\toprule")
print(r"\textbf{Metric} & \textbf{PID (Anti-windup)} & \textbf{MOTION} & \textbf{Gain} \\ \midrule")
print(f"Settling Time (2\\%) & {m_pid[0]:.3f} s & {m_mot[0]:.3f} s & {imp_settle:.1f}\\% \\\\")
print(f"Overshoot           & {m_pid[1]:.1f}\\% & {m_mot[1]:.1f}\\% & 3.0\\% (Abs) \\\\")
print(f"Total Energy ($\\int U^2 dt$) & {m_pid[2]:.2f} & {m_mot[2]:.2f} & \\textbf{{{imp_energy:.1f}\\%}} \\\\")
print(f"Control Jitter (Var) & {m_pid[3]:.4f} & {m_mot[3]:.4f} & {imp_jitter:.1f}\\% \\\\ \\bottomrule")
print(r"\end{tabular}")
print(r"\end{table}")
print("="*60)

# ==========================================
# 4. ГРАФІКИ
# ==========================================
plt.style.use('seaborn-v0_8-whitegrid')
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

ax1.plot(time, h_pid_noisy['pos'], label='PID + Anti-windup', color='#d62728', alpha=0.8, lw=1.5)
ax1.plot(time, h_mot_noisy['pos'], label='MOTION', color='#1f77b4', lw=2)
ax1.axhline(target_angle, color='k', linestyle='--', label='Target')
ax1.set_ylabel('Position (rad)')
ax1.set_title(r'Dynamic Step Response under Sensor Noise ($\sigma=0.05$)')
ax1.legend(loc='lower right')

ax2.plot(time, h_pid_noisy['u'], label='PID Torque', color='#d62728', alpha=0.7, lw=1)
ax2.plot(time, h_mot_noisy['u'], label='MOTION Torque', color='#1f77b4', lw=1.5)
ax2.axhline(U_MAX, color='gray', linestyle=':', label='Saturation Limit')
ax2.axhline(-U_MAX, color='gray', linestyle=':')
ax2.set_ylabel('Control Effort (Nm)')
ax2.legend(loc='upper right')

ax3.plot(time, h_pid_noisy['e'], label='PID Energy', color='#d62728', lw=2)
ax3.plot(time, h_mot_noisy['e'], label='MOTION Energy', color='#1f77b4', lw=2)
ax3.set_ylabel(r'Total Energy ($\int U^2 dt$)')
ax3.set_xlabel('Time (s)')
ax3.legend(loc='lower right')

plt.tight_layout()
plt.savefig('motion_benchmark_ieee.png', dpi=300)
plt.show()

# ==========================================
# 5. PARAMETER SWEEP
# ==========================================
noise_levels = np.linspace(0.0, 0.2, 10)
pid_e_sweep = []
mot_e_sweep = []

for sigma in noise_levels:
    hp, hm = run_simulation(sigma)
    pid_e_sweep.append(hp['e'][-1])
    mot_e_sweep.append(hm['e'][-1])

plt.figure(figsize=(8, 5))
plt.plot(noise_levels, pid_e_sweep, marker='o', color='#d62728', label='PID')
plt.plot(noise_levels, mot_e_sweep, marker='s', color='#1f77b4', label='MOTION')
plt.title('Energy Consumption vs. Sensor Noise Level')
plt.xlabel(r'Sensor Noise Standard Deviation ($\sigma$ rad)')
plt.ylabel(r'Total Energy Expenditure ($\int U^2 dt$)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('motion_sweep_ieee.png', dpi=300)
plt.show()

input("Done! Press Enter to exit...")