import simulation.quadrotor3 as quad
import simulation.config as cfg
import simulation.animation as ani
import matplotlib.pyplot as pl
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import random
from math import pi, sin, cos
import math
import gym
from gym import error, spaces, utils

class Circle_sensors(object):
    def __init__(self, r,level):
        self.n = 15 #Resolution of sensors
        cir = np.array([[cos(2*np.pi/self.n *x)*r, sin(2*np.pi/self.n *x)*r, 0] for x in range(self.n +1)])
        cir += np.array([[0.0, 0.0, 0.0] for x in range(self.n +1)])
        self.p1 = cir
        self.p1_adj = cir
        self.level = level

    ##Needs the quad data just for quarternion functions
    def update_sensors(self,xyz,iris):
        q = iris.get_q()
        Q_inv = iris.q_conj(q)
        Q =  iris.q_mult(Q_inv)
        r =  self.R(Q_inv)
        adj_p1 = np.einsum('ij,kj->ki', r, self.p1)
        adj_p1 = np.matlib.repmat(xyz.T, self.n+1,1)+adj_p1
        self.p1_adj = adj_p1

    def R(self, p):
        p0, p1, p2, p3 = p[0,0], p[1,0], p[2,0], p[3,0]
        x11 = p0**2+p1**2-p2**2-p3**2
        x12 = 2.*(p1*p2-p0*p3)
        x13 = 2.*(p1*p3+p0*p2)
        x21 = 2.*(p1*p2+p0*p3)
        x22 = p0**2-p1**2+p2**2-p3**2
        x23 = 2.*(p2*p3-p0*p1)
        x31 = 2.*(p1*p3-p0*p2)
        x32 = 2.*(p2*p3+p0*p1)
        x33 = p0**2-p1**2-p2**2+p3**2
        return np.array([[x11, x12, x13],
                        [x21, x22, x23],
                        [x31, x32, x33]])

    def get_points(self):
        return self.p1_adj

    def plot(self,axis):
        axis.plot(self.p1_adj[:,0], self.p1_adj[:,1], self.p1_adj[:,2],'k')

    def get_level(self):
        return self.level

    """
    As quartenions are used, the circle is actually single points that are rotated
    The distace to a random points (loc) is calculated as the distance from the closest point
    on a circle to this (loc) point
    -This is time consuming as more tha one sensor is used
    """
    def calculate_distance(self,loc):
        min_dist = 100
        for point in self.p1_adj:
            temp = np.array(point)
            dist = np.linalg.norm(loc-temp)
            if dist < min_dist:
                min_dist = dist
        return min_dist

"""
The object the quadcopter needs to avoid
"""
class Sphere:
    def __init__(self, *args):
        max_rad, length, width, height = args
        x = np.random.uniform(low=-length/2, high=length/2)
        y = np.random.uniform(low=-width/2, high=width/2)
        z = np.random.uniform(low=-height/2, high=height/2)
        self.rad = np.random.uniform(low=1, high=max_rad)
        self.xyz = np.vstack([x, y, z])
        
    def get_radius(self):
        return self.rad

    def get_center(self):
        return self.xyz

    def translate_sphere(self, vector):
        self.xyz += vector