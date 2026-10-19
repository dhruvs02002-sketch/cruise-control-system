""" To run python -m src/metrics.py """

from scripts.run_closed_loop import simulate_closed_loop

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

v_total = simulate_closed_loop(G, Cpid, t_array, v_ref_signal, d_signal)


def rise_time(v_total,t_array,V_REF):
    mask = t_array < 60.0
    v_startup = v_total[mask]
    t_startup = t_array[mask]

    v_10 = 0.10 * V_REF
    v_90 = 0.90 *   V_REF
    min_index = np.abs(v_startup - v_10).argmin()
    max_index = np.abs(v_startup - v_90).argmin()
    return t_startup[max_index] - t_startup[min_index]


def peak_overshoot(v_total, t_array, V_REF):
    mask = t_array < 60

    v_startup = v_total[mask]

    max_velo = np.max(v_startup)

    overshoot = max(0,((max_velo - V_REF)/V_REF)) * 100

    return max(0.0,float(overshoot))


def settling_time(v_total,t_array,V_REF,tol = 0.02):
    mask = t_array < 60.0
    v_startup = v_total[mask]
    t_startup = t_array[mask]

    mask1 = np.abs(v_startup - V_REF) > (tol * V_REF)
    outside_indices = np.where(mask1)[0]

    if len(outside_indices) == 0:
        return 0.0

    last_outside_idx = outside_indices[-1]

    if last_outside_idx == len(v_startup) - 1:
        return np.nan

    return float(t_startup[last_outside_idx + 1])


def steady_state_error(v_total,t_array,V_REF):
    mask1 = t_array < 60.0

    t_startup = t_array[mask1]
    v_startup = v_total[mask1]

    mask = np.abs(t_startup - 59.9).argmin()
    return float(np.abs(V_REF - v_startup[mask]))


def integral_absolute_error(v_total,t_array,V_REF):
    error = np.abs(V_REF - v_total)
    return float(np.trapz(error,t_array))





