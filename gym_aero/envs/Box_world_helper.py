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
class Sphere_Object:
    def __init__(self, r):
        resolution = 20

        self.rad = r
        u = np.linspace(0, 2 * np.pi, resolution)
        v = np.linspace(0, np.pi, resolution)


        self.x =random.randint(-25,25)/10.
        self.y =random.randint(-25,25)/10.
        self.z =random.randint(-25,25)/10.

        self.x_cir = r * np.outer(np.cos(u), np.sin(v)) + self.x
        self.y_cir = r * np.outer(np.sin(u), np.sin(v)) + self.y
        self.z_cir = r * np.outer(np.ones(np.size(u)), np.cos(v)) + self.z


    def get_radius(self):
        return self.rad

    def get_center(self):
        return [self.x,self.y,self.z]

    def move_x(self,amount):
        self.x = self.x + amount
        self.x_cir  = self.x_cir + self.x


    def move_y(self,amount):
        self.y = self.y + amount
        self.y_cir  = self.y_cir + self.y

    def move_z(self,amount):
        self.z = self.z + amount
        self.z_cir  = self.z_cir + self.z


    def plot_sphere(self,ax):
        ax.plot_surface(self.x_cir,self.y_cir,self.z_cir,  rstride=1, cstride=1, color='b', linewidth=0)
