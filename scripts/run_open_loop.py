""" To run python -m scripts.run_open_loop """
""" For first 60 seconds vehicle goes on a flat ground and after that 
    on an inclined plane """





import numpy as np
import control as ct
import matplotlib.pyplot as plt

from src.plant import get_vehicle_plant,calculate_gravity_disturbance
from config.parameters import DAMPING, V_REF

def open_loop_simulations():
    t_array = np.linspace(0,120,1200)

    tr_function = get_vehicle_plant()

    u_flat = DAMPING * V_REF

    disturbance = calculate_gravity_disturbance()
    u_hill = np.where(t_array >= 60.0,u_flat - disturbance,u_flat)

    frcd_rspns = ct.forced_response(tr_function,T = t_array,U = u_flat)
    v_flat = frcd_rspns.outputs

    frcd_rspns_hill = ct.forced_response(tr_function,T = t_array,U = u_hill)
    v_hill = frcd_rspns_hill.outputs
    print(v_hill)

    plt.figure(figsize=(10, 5))
    plt.plot(t_array, v_flat, 'g--', label='Flat Ground (Constant 1500 N Force)', alpha=0.7)
    plt.plot(t_array, v_hill, 'b-', label='Hit 5° Hill at t = 60s (Open Loop)', linewidth=2)
    plt.axhline(V_REF, color='r', linestyle=':', label=f'Target Speed ({V_REF} m/s)')
    plt.axvline(60.0, color='k', linestyle='--', alpha=0.5, label='Hill Starts (t = 60s)')

    plt.title('Open-Loop Vehicle Speed Step Response & Disturbance Impact')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

open_loop_simulations()


