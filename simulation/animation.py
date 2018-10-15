from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pl
from math import sin, cos, pi
import numpy as np
import numpy.matlib

class Visualization:
    def __init__(self, aircraft, n, quaternion=False):
        self.aircraft = aircraft
        self.r = aircraft.prop_radius
        self.l = aircraft.l
        self.n = n

        self.p1 = np.array([[cos(2*pi/n*x)*self.r, sin(2*pi/n*x)*self.r, 0] for x in range(n+1)])
        self.p2 = np.array([[cos(2*pi/n*x)*self.r, sin(2*pi/n*x)*self.r, 0] for x in range(n+1)])
        self.p3 = np.array([[cos(2*pi/n*x)*self.r, sin(2*pi/n*x)*self.r, 0] for x in range(n+1)])
        self.p4 = np.array([[cos(2*pi/n*x)*self.r, sin(2*pi/n*x)*self.r, 0] for x in range(n+1)])

        self.p1 += np.array([[self.l, 0.0, 0.0] for x in range(n+1)])
        self.p2 += np.array([[0.0, -self.l, 0.0] for x in range(n+1)])
        self.p3 += np.array([[-self.l, 0.0, 0.0] for x in range(n+1)])
        self.p4 += np.array([[0.0, self.l, 0.0] for x in range(n+1)])

        if quaternion:
            self.x_i = np.array([[0.],
                                [1.],
                                [0.],
                                [0.]])
            self.y_i = np.array([[0.],
                                [0.],
                                [1.],
                                [0.]])
            self.z_i = np.array([[0.],
                                [0.],
                                [0.],
                                [1.]])
            self.q_mult = self.aircraft.q_mult
            self.q_conj = self.aircraft.q_conj

            self.zero = np.array([[0.]])
        
        self.crashed = False

    def draw3d(self, ax):
        xyz, R = self.aircraft.xyz, self.aircraft.R1(self.aircraft.zeta).T

        ax.scatter(xyz[0,0], xyz[1,0], xyz[2,0], color='black')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], R[0,0], R[0,1], R[0,2], pivot='tail', color='red')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], R[1,0], R[1,1], R[1,2], pivot='tail', color='green')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], R[2,0], R[2,1], R[2,2], pivot='tail', color='blue')

        # rotate to aircraft attitude
        p1 = np.einsum('ij,kj->ik', self.p1, R.T)
        p2 = np.einsum('ij,kj->ik', self.p2, R.T)
        p3 = np.einsum('ij,kj->ik', self.p3, R.T)
        p4 = np.einsum('ij,kj->ik', self.p4, R.T)

        # shift to z
        p1 = np.matlib.repmat(xyz.T,self.n+1,1)+p1
        p2 = np.matlib.repmat(xyz.T,self.n+1,1)+p2
        p3 = np.matlib.repmat(xyz.T,self.n+1,1)+p3
        p4 = np.matlib.repmat(xyz.T,self.n+1,1)+p4
        P = (p1, p2, p3, p4)
        for p in P:
            for n in p:
                if n[2]<0:
                    self.crashed = True

        ax.plot(p1[:,0], p1[:,1], p1[:,2],'k')
        ax.plot(p2[:,0], p2[:,1], p2[:,2],'k')
        ax.plot(p3[:,0], p3[:,1], p3[:,2],'k')
        ax.plot(p4[:,0], p4[:,1], p4[:,2],'k')


    def draw3d_quat(self, ax):
        xyz, q = self.aircraft.state[0:3], self.aircraft.state[3:7]
        Q_inv = self.q_conj(q)
        Q = self.q_mult(Q_inv)
        r = self.R(Q_inv)
        x_b = Q.dot(self.q_mult(self.x_i).dot(q))
        y_b = Q.dot(self.q_mult(self.y_i).dot(q))
        z_b = Q.dot(self.q_mult(self.z_i).dot(q))

        ax.scatter(xyz[0,0], xyz[1,0], xyz[2,0], color='black')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], x_b[1,0], x_b[2,0], x_b[3,0], pivot='tail', color='red')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], y_b[1,0], y_b[2,0], y_b[3,0], pivot='tail', color='green')
        ax.quiver(xyz[0,0], xyz[1,0], xyz[2,0], z_b[1,0], z_b[2,0], z_b[3,0], pivot='tail', color='blue')

        # rotate to aircraft attitude
        p1 = np.einsum('ij,kj->ki', r, self.p1)
        p2 = np.einsum('ij,kj->ki', r, self.p2)
        p3 = np.einsum('ij,kj->ki', r, self.p3)
        p4 = np.einsum('ij,kj->ki', r, self.p4)

        # shift to xyz
        p1 = np.matlib.repmat(xyz.T, self.n+1,1)+p1
        p2 = np.matlib.repmat(xyz.T, self.n+1,1)+p2
        p3 = np.matlib.repmat(xyz.T, self.n+1,1)+p3
        p4 = np.matlib.repmat(xyz.T, self.n+1,1)+p4

        # plot rotated
        ax.plot(p1[:,0], p1[:,1], p1[:,2],'k')
        ax.plot(p2[:,0], p2[:,1], p2[:,2],'k')
        ax.plot(p3[:,0], p3[:,1], p3[:,2],'k')
        ax.plot(p4[:,0], p4[:,1], p4[:,2],'k')

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

    def draw_goal(self, ax, goal, color='green'):
        ax.scatter(goal[0,0], goal[1,0], goal[2,0], color=color)

    # Draws a line between p1 and p2 (3d vectors) in the given color
    def draw_line(self, ax, p1, p2, color=[0,0,0]):
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]],zs=[p1[2],p2[2]], color=color, lw=1)
    
    def set_wall(self):
        pass

    def draw_wall(self, point, normal, size, offset):
        pass
