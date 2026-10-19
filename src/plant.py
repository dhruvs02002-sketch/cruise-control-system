import control
import numpy as np 
from config.parameters import MASS,DAMPING,GRAVITY,HILL_ANGLE_DEG

def get_vehicle_plant(m = MASS,b = DAMPING):
    N_S = [1.0]                 # Numerator
    D_S = [m,b]                 # Denominator

    tr = control.tf(N_S,D_S)    # Constructs the transfer function
    return tr

def calculate_gravity_disturbance(theta = HILL_ANGLE_DEG,m = MASS,g = GRAVITY):
    theta_rad = theta * np.pi / 180
    sin_theta_rad = np.sin(theta_rad)
    return (m * g * sin_theta_rad)


    



