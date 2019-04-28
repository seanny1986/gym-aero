import simulation.animation_gl as ani_gl
import simulation.config as cfg
import simulation.quadrotor3 as quad
import numpy as np
from math import pi
import time

params = cfg.params
iris = quad.Quadrotor(params)
thetas = np.linspace(0, pi, 50, endpoint=True)

xs = np.cos(thetas)
ys = np.sin(thetas)
zs = np.zeros(thetas.shape)

xyz = np.vstack([xs,ys,zs]).T

ani = ani_gl.VisualizationGL(name="Test")
init_rendering = True

t_end = time.time() + 60

while time.time() < t_end:
    ani.draw_quadrotor(iris)
    for pt in xyz:
        ani.draw_goal(pt.reshape(3,1))
    ani.draw()