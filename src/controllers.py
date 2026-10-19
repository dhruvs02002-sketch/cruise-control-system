import control

from config.parameters import KP_DEFAULT,KI_DEFAULT,KD_DEFAULT

def get_p_controller(Kp = KP_DEFAULT):
    num = [Kp]
    den = [1.0]
    return control.tf(num,den)

def get_pi_controller(Kp = KP_DEFAULT,Ki = KI_DEFAULT):
    num = [Kp,Ki]
    den = [1.0,0.0]
    return control.tf(num,den)

def get_pid_controller(Kp = KP_DEFAULT,Ki = KI_DEFAULT,Kd = KD_DEFAULT):
    num = [Kd,Kp,Ki]
    den = [1.0,0.0]
    return control.tf(num,den)

