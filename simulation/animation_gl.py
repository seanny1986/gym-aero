import pyglet
from pyglet.gl import *
import ratcave as rc
import time
import numpy as np
import os

from pyglet.window import key
from pyglet.window import mouse

def length(vec):
    s = 0;
    for x in vec:
        s += x*x;

    return np.sqrt(s);

def midpoint(pt1, pt2):
    assert len(pt1) == len(pt2);

    mid = [];
    for i in range(len(pt1)):
        mid_i = (pt1[i] + pt2[i]) / 2.0;
        mid.append(mid_i);

    return mid;

def normalize(vec):
    leng = length(vec);
    return (x/leng for x in vec);

class VisualizationGL:

    def __init__(self, name=None, width=640, height=480):
        print(dir(rc.resources));
        # texPath = rc.resources.img_uvgrid;
        texPath = rc.resources.img_white;
        # texPath = os.path.join(os.path.dirname(os.path.realpath(__file__)), "res", "grid.jpg");
        # print();
        self.texture = rc.Texture.from_image(texPath);
        self.__init_entities();
        self.__init_window(width, height, name);

    #Draws the scene, should be called once per frame
    def draw(self):
        pyglet.clock.tick();
        self.window.switch_to();
        self.window.dispatch_events()
        self.window.dispatch_event('on_draw')
        self.window.flip();

    #Draw a quadrotor
    def draw_quadrotor(self, quad):
        quad_entity = self.quad_pool.get();
        axis_entity = self.axis_pool.get();

        xyz, zeta, _, _ = quad.get_state();
        pos = self.__trans_pos(xyz);
        rot = self.__trans_rot(zeta);

        quad_entity.position.xyz = pos;
        quad_entity.rotation.xyz = rot;
        axis_entity.position.xyz = pos;
        axis_entity.rotation.xyz = rot;
        axis_entity.scale = .3;

        self.world.add_children(quad_entity);
        self.world.add_children(axis_entity);

    #Draw a goal position
    def draw_goal(self, pos, color=(0, 0.5, 0)):
        goal_entity = self.goal_pool.get();
        goal_entity.position.xyz = self.__trans_pos(pos);
        goal_entity.uniforms['diffuse'] = color;

        self.world.add_children(goal_entity);

    #Draws a text label on the screen
    def draw_label(self, text, pos, anchor_x='center', anchor_y='center'):
        label = pyglet.text.Label(text,
                          font_name='Times New Roman',
                          font_size=12,
                          x=pos[0], y=pos[1],
                          anchor_x=anchor_x, anchor_y=anchor_y);
        self.labels.append(label);

    def draw_line(self, pt1, pt2, color=(0, 0.5, 0)):
        line_entity = self.line_pool.get();
        dist = length((pt2-pt1).ravel());

        x_hat = np.array([[1],[0],[0]]);
        y_hat = np.array([[0],[1],[0]]);
        z_hat = np.array([[0],[0],[1]]);
        pt1_hat = pt1 / np.linalg.norm(pt1);
        pt2_hat = pt2 / np.linalg.norm(pt2);

        # y_angle = np.acos(np.dot(y_hat, ))

        line_entity.rotation = (pt1_hat).ravel();
        line_entity.rotation = (45,45,45);

        pt1 = self.__trans_pos(pt1);
        pt2 = self.__trans_pos(pt2);
        midpt = midpoint(pt1, pt2);      

        line_entity.scale = (0.05, dist/2, 0.05);
        line_entity.position = midpt;

        self.world.add_children(line_entity);

    #Translates a simulation position into an OpenGL position
    def __trans_pos(self, xyz):
        return xyz[0,0],xyz[2,0],xyz[1,0];

    #Translate a simulation rotation into an OpenGL rotation
    def __trans_rot(self, zeta):
        rot = np.degrees(zeta.ravel());
        rot[0] *= -1;
        rot[0] += 90.0;
        return rot;

    def __make_line(self):
        line = self.obj_reader.get_mesh("Cylinder", position=(0,0,0), scale=.5)

        return line;

    #Creates the quadrotor model
    def __make_quadrotor(self):
        toruses = [self.obj_reader.get_mesh("Torus", position=(0,0.5,0), scale=.3),
                   self.obj_reader.get_mesh("Torus", position=(0,-0.5,0), scale=.3),
                   self.obj_reader.get_mesh("Torus", position=(0.5,0,0), scale=.3),
                   self.obj_reader.get_mesh("Torus", position=(-0.5,0,0), scale=.3)];

        quad = rc.EmptyEntity(name='quadrotor');

        for torus in toruses:
            torus.uniforms['diffuse'] = (0.0,0.0,0.0);
            quad.add_children(torus);
        quad.scale=.5

        return quad;

    #Creates the goal model
    def __make_goal(self):
        goal = self.obj_reader.get_mesh("Sphere");
        goal.scale = 0.09;
        
        return goal;

    #Creates the arrow model
    def __make_arrow(self, color):
        arrow_top = self.obj_reader.get_mesh("Cone");
        arrow_bottom = self.obj_reader.get_mesh("Cylinder");

        arrow_bottom.scale.xyz = 0.6,0.5,0.6;
        arrow_top.scale.xyz = 1.0,0.5,1.0;

        arrow_bottom.position.xyz =0,0.5,0; 
        arrow_bottom.uniforms['diffuse'] = color;
        arrow_top.position.xyz=0,0.75,0;
        arrow_top.uniforms['diffuse'] = color;

        arrow = rc.EmptyEntity();
        arrow.add_children(arrow_top, arrow_bottom);

        return arrow;

    #Creates the axis model
    def __make_axis(self):
        axis = rc.EmptyEntity(name='arrow');

        x_axis_arrow = self.__make_arrow(color=(1,0,0));
        y_axis_arrow = self.__make_arrow(color=(0,1,0));
        z_axis_arrow = self.__make_arrow(color=(0,0,1));

        x_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;
        y_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;
        z_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;

        x_axis_arrow.rotation.xyz = 0.0,   0.0, -90.0;
        y_axis_arrow.rotation.xyz = 0.0,   0.0, 180.0;
        z_axis_arrow.rotation.xyz = -90.0, 0.0, 0.0  ;

        axis.add_children(x_axis_arrow, y_axis_arrow, z_axis_arrow);
        return axis;

    def __make_grid(self):
        plane = self.obj_reader.get_mesh("Plane");
        plane.scale = 6.0;
        color = (1,1,1);
        plane.position.xyz = 0,0,0;
        plane.uniforms['diffuse'] = color;
        plane.uniforms['spec_weight'] = 0;
        plane.textures.append(self.texture);

        return plane;
        # rng = 3;
        # grid = rc.EmptyEntity();
        # for x in range(-rng, rng):
        #     for y in range(-rng, rng):
        #         plane = self.obj_reader.get_mesh("Plane");
        #         plane.scale = 1.5;

        #         # if(abs(x + y) % 2 == 0):
        #         #     color = (0.5,0.5,0.5);
        #         # else:
        #         color = (1,0,0);

        #         plane.position.xyz = x + 1.5,y + 1.5,0;
        #         plane.uniforms['diffuse'] = color;
        #         plane.uniforms['spec_weight'] = 0;
        #         plane.textures.append(self.texture);
        #         grid.add_children(plane);

        # return grid;

    #Initialize entity pools
    def __init_entities(self):
        self.obj_filename = rc.resources.obj_primitives;
        self.obj_reader = rc.WavefrontReader(self.obj_filename);

        self.quad_pool = EntityPool(self.__make_quadrotor);
        self.axis_pool = EntityPool(self.__make_axis);
        self.goal_pool = EntityPool(self.__make_goal);
        self.grid_pool = EntityPool(self.__make_grid);
        self.line_pool = EntityPool(self.__make_line);

        self.entity_pools = [self.quad_pool,
                            self.axis_pool,
                            self.goal_pool,
                            self.grid_pool,
                            self.line_pool];

    #Resets entity pools
    def __reset_drawing(self):
        #Reset entity pools for entity re-use
        [pool.reset() for pool in self.entity_pools];

        # self.world.children = [];
        [self.world.remove_children(x) for x in self.world.children];

        grid_x = self.grid_pool.get();
        grid_y = self.grid_pool.get();
        grid_z = self.grid_pool.get();
        grid_x_back = self.grid_pool.get();
        grid_y_back = self.grid_pool.get();
        grid_z_back = self.grid_pool.get();

        rng = 6.0;

        grid_x.position.x = -rng;
        grid_y.position.y = -rng;
        grid_z.position.z = -rng;
        grid_x_back.position.x = rng;
        grid_y_back.position.y = rng;
        grid_z_back.position.z = rng;

        grid_x.rotation.y = 90;
        grid_y.rotation.x = -90;
        grid_x_back.rotation.y = -90;
        grid_y_back.rotation.x = 90;
        grid_z_back.rotation.x = 180;

        # grid_z.rotation.x = 90;

        self.world.add_children(grid_x, grid_y, grid_z, grid_x_back, grid_y_back, grid_z_back);

        self.labels = [];

    #Initialize the  & scene
    def __init_window(self, width, height, name):
        self.window = pyglet.window.Window(width=width, height=height, caption=name);
        self.world = rc.EmptyEntity(name='world');
        self.world.position.xyz = 0, 0, -2;
        self.world.scale = 0.5;
        self.grid = self.__make_grid();
        self.scene = rc.Scene(meshes=self.world);
        self.scene.camera.position.xyz = 0,0,2;
        self.world.rotation.xyz = 45, 45, 0;
        self.__reset_drawing();

        #This is called on the draw event
        @self.window.event
        def on_draw():
            with rc.default_shader:
                self.scene.draw();

            [lbl.draw() for lbl in self.labels];
            self.__reset_drawing();

        #This is called during a mouse drag event
        @self.window.event
        def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
            rot_speed = 1.0;
            self.world.rotation.y += dx * rot_speed;
            self.world.rotation.x -= dy * rot_speed;

        
        #Register keyboard handler        
        keys = key.KeyStateHandler()
        self.window.push_handlers(keys)

        def on_key_press(dt):
            camera_speed = 3
            if keys[key.W]:
                self.scene.camera.position.z -= camera_speed * dt
            if keys[key.S]:
                self.scene.camera.position.z += camera_speed * dt
            if keys[key.R]:
                self.scene.camera.position.xyz = 0,0,0;
                self.world.rotation.xyz = 0,0,0;

        #Processes key presses on a clock schedule
        pyglet.clock.schedule(on_key_press);
           

class EntityPool:
    def __init__(self, gen_func):
        self.entities = [];
        self.entitiesUsed = 0;
        self.gen_func = gen_func;

    def reset(self):
        self.entitiesUsed = 0;

    def get(self):
        if(len(self.entities) <= self.entitiesUsed):
            entity = self.gen_func();
            self.entities.append(entity);
        else:
            entity = self.entities[self.entitiesUsed];

        self.entitiesUsed += 1;
        return entity;
