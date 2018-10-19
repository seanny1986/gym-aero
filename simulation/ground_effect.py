import numpy as np
import simulation.config as cfg
from math import sin, pi

class GroundEffect:
    def __init__(self, aircraft, ground=0.):
        self.aircraft = aircraft
        self.l = aircraft.l
        self.prop_radius = aircraft.prop_radius
        self.GROUND_NORMAL = np.array([[0.],
                                    [0.],
                                    [1.]])
        self.GROUND_EFFECT_HEIGHT = self.l+self.prop_radius

    def ground_effect(self, rpm, pos):
        xyz, _, _, _ = pos
        ac_norm = self.aircraft.get_body_norm()
        effect = 1.-ac_norm.T.dot(self.GROUND_NORMAL)
        g_eff = (sin(pi*xyz[2]/2/self.GROUND_EFFECT_HEIGHT)+1)*effect/2
        return g_eff, g_eff*ac_norm