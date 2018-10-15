import numpy as np
import simulation.config as cfg

params = cfg.params
l = params["l"]
prop_radius = params["prop_radius"]

GROUND_NORMAL = np.array([[0.],
                        [0.],
                        [1.]])
GROUND_EFFECT_HEIGHT = l+prop_radius

lambda eff : 1

def ground_effect(rpm, pos):
    xyz, zeta, uvw, qr = pos
    ac_norm = xyz/np.linalg.norm(xyz)
    cos_theta = ac_norm.T.dot(GROUND_NORMAL)
    g_eff = 0.
    if xyz[2] <= GROUND_EFFECT_HEIGHT:
        g_eff += eff(xyz[2])*cos_theta
    return g_eff
