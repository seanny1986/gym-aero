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

"""
Box world object, this creates boxes for the quad to avoid,
the boxes are always within the env, and will not block a goal or be drawn on
the quad. The boxes are cubes for now, this can be changed in later versions
"""
class Box():

    def __init__(self,size):

        self.x = random.randint(-29,29)/10.
        self.y = random.randint(-29,29)/10.
        self.z = random.randint(-29,29)/10.
        #size = 0.5#random.randint(5,10)/30.
        self.size = size

        self.gen(self.x,self.y,self.z,size)

    def gen(self,x,y,z,size):
        A1 = np.array([x,y,z])
        A2 = np.array([x,y+size,z])
        A3 = np.array([x+size,y+size,z])
        A4 = np.array([x+size,y,z])

        A5 = np.array([x,y,z+size])
        A6 = np.array([x,y+size,z+size])
        A7 = np.array([x+size,y+size,z+size])
        A8 = np.array([x+size,y,z+size])

        ##All the vertices of the cube
        self.vert_list = [A1,A2,A3,A4,A5,A6,A7,A8]

        self.center = np.array([x+size*0.5,y+size*0.5,z+size*0.5])

        self.faces =[[A1,A2,A3,A4],[A5,A6,A7,A8],[A1,A2,A6,A5],[A2,A3,A7,A6],
                    [A3,A4,A8,A7],[A4,A1,A5,A8]]
        self.x_points =[]
        self.y_points =[]
        self.z_points =[]


        for v in self.faces:
            self.x_points.append(v[0])
            self.y_points.append(v[1])
            self.z_points.append(v[2])

    def get_verts(self):
        return self.vert_list

    def get_center(self):
        return self.center

    ##Move the box in the given directions
    def move(self,x,y,z):
        self.gen(self.x+x,self.y+y,self.z+z,self.size)

    ##Returns the maximum distance from the center to the corner of the box
    def get_distance_to_corner(self):
        dif = self.center - self.vert_list[0]
        dif = np.linalg.norm(dif)
        return dif

    #returns a vector that is normal to the face
    def get_normal(self,face):
        V1 = face[0] - face[1]
        V2 = face[1] - face[2]
        n = np.cross(V1,V2)
        return n

    def get_faces(self):
        return self.faces

    ##Checks if a point lines on any of the surfaces of the cube
    def check_on_surface(self,p):
        res = []
        for vert in self.verts:
            v1,v2,v3,v4 = vert
            matrix = np.array([[v1[0],v2[0],v3[0],p[0]],
            [v1[1],v2[1],v3[1],p[1]],
            [v1[2],v2[2],v3[2],p[2]],
            [1,1,1,1]])
            det = np.linalg.det(matrix)
            if det != 0:
                res.append(False)
            else:
                res.append(True)

    def plot_verts(self,axis):
        axis.scatter(self.x_points,self.y_points,self.z_points)

    ##draws the box
    def draw_cube(self,axis):
        axis.add_collection3d(Poly3DCollection(self.faces,
        facecolors='black', linewidths=1, edgecolors='r', alpha=.25))



"""--------------------------------------------------------------
These are the various kind of sensors being used
The first kind is the simplest. It has 8 points laid in a circle around the
quad, it calculates the distance from each of the circles to the nearest face of a each
cube. This is then returned as the the reward.
The sensors rotate, in pitch yaw and role with the quad
all angles are in rad, the reward is scaled to 0-10
    o  o  o
     \ | /
  o___O O___o
      O O
     / | \
    o  o  o
"""


class Circle_sensors():

    def __init__(self):

        self.num_sensors = 8
        self.ang_spacing = (2*np.pi)/self.num_sensors
        self.sensor_length = 1
        self.sensors = []
        xyz = np.array([0,0,0])
        base_s = np.array([xyz[0]+self.sensor_length/2,xyz[1],0])
        base_e = np.array([xyz[0]-self.sensor_length/2,xyz[1],0])
        self.sensors.append([base_s,base_e])

        ##Creates sensors at equal intervals around the base sensor
        for i in range(1,self.num_sensors):
            vec_s_x = math.cos(self.ang_spacing*i)*base_s[0] - math.sin(self.ang_spacing*i)*base_s[1]
            vec_s_y = math.sin(self.ang_spacing*i)*base_s[0] + math.cos(self.ang_spacing*i)*base_s[1]
            vec_s_z = xyz[2]
            vec_e_x = -vec_s_x
            vec_e_y = -vec_s_y

            vec_e_z = xyz[2]
            temp_vec_s = np.array([vec_s_x,vec_s_y,vec_s_z])
            temp_vec_e = np.array([vec_e_x,vec_e_y,vec_e_z])
            self.sensors.append([temp_vec_s,temp_vec_e])


    #Updates location fo sensors
    def update_sensor_location(self,quad_xyz,quad_zeta):
        for i in range(0,len(self.sensors)):
            self.sensors[i][0] = self.sensors[i][0] + quad_xyz
            self.sensors[i][1] = self.sensors[i][1] + quad_xyz
            self.sensors[i] = [self.sensors[i][0],self.sensors[i][1]]


        ##TODO:NEED TO solve this
        self.rotate(quad_zeta[2]) ##yaw
        self.translate_z(quad_zeta[1],0,quad_xyz)
        self.translate_z(quad_zeta[0],1,quad_xyz)

    ##Rotates the sensors in a cirle around one axis, ie. spining around
    def rotate(self,ror_angle):
        if ror_angle != np.pi/2:
            base_s,base_e = self.sensors[0]
            base_s[0] = math.cos(ror_angle)*base_s[0] - math.sin(ror_angle)*base_s[1]
            base_s[1] = math.sin(ror_angle)*base_s[0] + math.cos(ror_angle)*base_s[1]
            base_e[0] = -base_s[0]
            base_e[1] = - base_s[1]
            for i in range(1,self.num_sensors):
                vec_s,vec_e = self.sensors[i]
                vec_s[0] = math.cos(self.ang_spacing*i)*base_s[0] - math.sin(self.ang_spacing*i)*base_s[1]
                vec_s[1] = math.sin(self.ang_spacing*i)*base_s[0] + math.cos(self.ang_spacing*i)*base_s[1]
                vec_e[0] = -vec_s[0]
                vec_e[1]= -vec_s[1]
                self.sensors[i] =[np.array(vec_s),np.array(vec_e)]
        else:
            for i in range(0,self.num_sensors):
                vec_s,vec_e = self.sensors[i]
                temp = vec_s[0]
                vec_s[0] = vec_s[1]
                vec_s[1] = temp

                temp = vec_e[0]
                vec_e[0]= vec_e[1]
                vec_e[1] = temp
                self.sensors[i] = [np.array(vec_s),np.array(vec_e)]

    ##Translates in xz ior yz
    #Can only trake in x or y i.e 0 or 1 for the index
    ##D1 is dimensions index,
    def translate_z(self,ang,d1,xyz):
        if (d1 > 1):
            raise ValueError('Dimension was not x or y')

        ##Not a multiple of 90deg
        if ang % (np.pi/2) != 0:
            for i in range(0,len(self.sensors)):
                vec_s,vec_e = self.sensors[i]
                vec_s[d1] = math.cos(ang)*vec_s[d1] - math.sin(ang)*vec_s[2]
                vec_s[2] = math.sin(ang)*vec_s[d1] + math.cos(ang)*vec_s[2]
                vec_e[d1] = -vec_s[d1]
                vec_e[2]= -vec_s[2]
                self.sensors[i] = [np.array(vec_s),np.array(vec_e)]
        else:
            if d1 == 1:
                base_s = np.array([xyz[0],0,xyz[2]+self.sensor_length /2])
                base_e = np.array([xyz[0],0,xyz[2]-self.sensor_length /2])
                d1_n = 0
            else:
                base_s = np.array([0,xyz[1],xyz[2]+self.sensor_length /2])
                base_e = np.array([0,xyz[1],xyz[2]-self.sensor_length /2])
                d1_n = 1

            self.sensors[0] = [base_s,base_e]
            for i in range(1,len(self.sensors)):
                vec_s,vec_e = self.sensors[i]
                vec_s[2] = math.cos(self.ang_spacing*i)*base_s[2] - math.sin(self.ang_spacing*i)*base_s[d1_n]
                vec_s[d1_n] = math.sin(self.ang_spacing*i)*base_s[2] + math.cos(self.ang_spacing*i)*base_s[d1_n]
                vec_e[d1] = 0
                vec_s[d1] =0
                vec_e[2] = -vec_s[2]
                vec_e[d1_n]= -vec_s[d1_n]
                self.sensors[i] = [np.array(vec_s),np.array(vec_e)]

    def draw_sensors(self,axis3d):
        for s in self.sensors:
            vs= s[0]
            ve=s[1]
            axis3d.scatter(vs[0],vs[1],vs[2],'r')
            axis3d.scatter(ve[0],ve[1],ve[2],'r')
            #axis3d.plot([vs[0],ve[0]],[vs[1],ve[1]],[vs[2],ve[2]])
        ##Returns the reward, based on the sensors with the top 3
        ##sensors disctances to the sum of all vertex of a cube
        # def get_reward(self,box_list):
        #     for box in box_list:
        #         closest_dist = 0
        #         for sensor in sensors:
        #             for vert in box.get_verts():

##Just for debuig
if __name__=='__main__':
    c = Circle_sensors()
    c.update_sensor_location(np.array([0.,0.,0.]),np.array([np.pi/6.,np.pi/6.,0.]))
    fig = pl.figure("Flying Skills")
    axis3d = fig.add_subplot(111, projection='3d')
    axis3d.set_xlim(-3, 3)
    axis3d.set_ylim(-3, 3)
    axis3d.set_zlim(-3, 3)
    axis3d.set_xlabel('West/East [m]')
    axis3d.set_ylabel('South/North [m]')
    axis3d.set_zlabel('Down/Up [m]')
    c.draw_sensors(axis3d)
    pl.show()

"""
Sensors for some form of ray tracing, using ports to Cpp
"""
