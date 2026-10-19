""" To run python -m scripts.run_closed_loop """
""" For next 60 seconds vehicle goes on a inclined ground """

import os
import control
import numpy as np
import matplotlib.pyplot as plt

from src.plant import get_vehicle_plant, calculate_gravity_disturbance
from src.controllers import get_p_controller, get_pi_controller, get_pid_controller
from config.parameters import V_REF,DAMPING

G = get_vehicle_plant()
D = calculate_gravity_disturbance()
Cp = get_p_controller()
Cpi = get_pi_controller()
Cpid = get_pid_controller()
u_flat = V_REF * DAMPING
t_array = np.linspace(0,120,1200)
v_ref_signal = np.full_like(t_array,V_REF)

d_signal = np.where(t_array >= 60,D,0.0)

def simulate_closed_loop(G, C, t_array, v_ref_signal, d_signal):
    t_ref = control.feedback(C * G,1)
    t_dist = -control.feedback(G,C)

    v_ref = control.forced_response(t_ref,T = t_array,U = v_ref_signal)
    v_dist = control.forced_response(t_dist,T = t_array,U = d_signal)

    v_total = v_ref.outputs + v_dist.outputs
    return v_total

v_p = simulate_closed_loop(G, Cp, t_array, v_ref_signal, d_signal)
v_pi = simulate_closed_loop(G, Cpi, t_array, v_ref_signal, d_signal)
v_pid = simulate_closed_loop(G, Cpid, t_array, v_ref_signal, d_signal)

# Create plot
plt.figure(figsize=(10, 5))

# Plot performance curves
plt.plot(t_array, v_p, 'r--', label='P Controller', alpha=0.8)
plt.plot(t_array, v_pi, 'g-.', label='PI Controller', alpha=0.8)
plt.plot(t_array, v_pid, 'b-', label='PID Controller', linewidth=2)

# Reference markers
plt.axhline(V_REF, color='k', linestyle=':', label=f'Target Speed ({V_REF} m/s)')
plt.axvline(60.0, color='gray', linestyle='--', alpha=0.6, label='Hill Starts (t = 60s)')

# Formatting
plt.title('Closed-Loop Speed Control: P vs PI vs PID Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.grid(True)
plt.legend()
plt.tight_layout()


os.makedirs('assets', exist_ok=True)
plt.savefig('assets/closed_loop_comparison.png', dpi=300)
plt.show()