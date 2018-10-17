# import pyglet
# from pyglet.gl import *
# import ratcave as rc
# import time
# import numpy as np
#
# from pyglet.window import key
# from pyglet.window import mouse
#
# class VisualizationGL:
#
#     def __init__(self, name=None, width=640, height=480):
#         self.__init_entities();
#         self.__init_window(width, height, name);
#
#     def draw(self):
#         pyglet.clock.tick();
#         self.window.switch_to();
#         self.window.dispatch_events()
#         self.window.dispatch_event('on_draw')
#         self.window.flip();
#
#     def draw_quadrotor(self, quad):
#         quad_entity = self.quad_pool.get();
#         axis_entity = self.axis_pool.get();
#
#         xyz, zeta, _, _ = quad.get_state();
#         pos = self.__trans_pos(xyz);
#         rot = self.__trans_rot(zeta);
#
#         quad_entity.position.xyz = pos;
#         quad_entity.rotation.xyz = rot;
#         axis_entity.position.xyz = pos;
#         axis_entity.rotation.xyz = rot;
#         axis_entity.scale = .3;
#
#         self.world.add_children(quad_entity);
#         self.world.add_children(axis_entity);
#
#     def draw_goal(self, pos, color=(0, 0.5, 0)):
#         goal_entity = self.goal_pool.get();
#         goal_entity.position.xyz = self.__trans_pos(pos);
#         goal_entity.uniforms['diffuse'] = color;
#
#         self.world.add_children(goal_entity);
#
#     def draw_label(self, text, pos, anchor_x='center', anchor_y='center'):
#         label = pyglet.text.Label(text,
#                           font_name='Times New Roman',
#                           font_size=12,
#                           x=pos[0], y=pos[1],
#                           anchor_x=anchor_x, anchor_y=anchor_y);
#         self.labels.append(label);
#
#     def __trans_pos(self, xyz):
#         return xyz[0,0],xyz[2,0],xyz[1,0];
#
#     def __trans_rot(self, zeta):
#         rot = np.degrees(zeta.ravel());
#         rot[0] *= -1;
#         rot[0] += 90.0;
#         return rot;
#
#     def __make_quadrotor(self):
#         toruses = [self.obj_reader.get_mesh("Torus", position=(0,0.5,0), scale=.3),
#                    self.obj_reader.get_mesh("Torus", position=(0,-0.5,0), scale=.3),
#                    self.obj_reader.get_mesh("Torus", position=(0.5,0,0), scale=.3),
#                    self.obj_reader.get_mesh("Torus", position=(-0.5,0,0), scale=.3)];
#
#         quad = rc.EmptyEntity(name='quadrotor');
#
#         for torus in toruses:
#             torus.uniforms['diffuse'] = (0.0,0.0,0.0);
#             quad.add_children(torus);
#         quad.scale=.5
#
#         return quad;
#
#     def __make_goal(self):
#         goal = self.obj_reader.get_mesh("Sphere");
#         goal.scale = 0.09;
#
#         return goal;
#
#     def __make_arrow(self, color):
#         arrow_top = self.obj_reader.get_mesh("Cone");
#         arrow_bottom = self.obj_reader.get_mesh("Cylinder");
#
#         arrow_bottom.scale.xyz = 0.6,0.5,0.6;
#         arrow_top.scale.xyz = 1.0,0.5,1.0;
#
#         arrow_bottom.position.xyz =0,0.5,0;
#         arrow_bottom.uniforms['diffuse'] = color;
#         arrow_top.position.xyz=0,0.75,0;
#         arrow_top.uniforms['diffuse'] = color;
#
#         arrow = rc.EmptyEntity();
#         arrow.add_children(arrow_top, arrow_bottom);
#
#         return arrow;
#
#     def __make_axis(self):
#         axis = rc.EmptyEntity(name='arrow');
#
#         x_axis_arrow = self.__make_arrow(color=(1,0,0));
#         y_axis_arrow = self.__make_arrow(color=(0,1,0));
#         z_axis_arrow = self.__make_arrow(color=(0,0,1));
#
#         x_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;
#         y_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;
#         z_axis_arrow.scale.xyz = 0.2, 2.0, 0.2;
#
#         x_axis_arrow.rotation.xyz = 0.0,   0.0, -90.0;
#         y_axis_arrow.rotation.xyz = 0.0,   0.0, 180.0;
#         z_axis_arrow.rotation.xyz = -90.0, 0.0, 0.0  ;
#
#         axis.add_children(x_axis_arrow, y_axis_arrow, z_axis_arrow);
#         return axis;
#
#     def __make_grid(self):
#         rng = 3;
#         grid = rc.EmptyEntity();
#         for x in range(-rng, rng):
#             for y in range(-rng, rng):
#                 plane = self.obj_reader.get_mesh("Plane");
#                 plane.scale = .5;
#
#                 if(abs(x + y) % 2 == 0):
#                     color = (0.5,0.5,0.5);
#                 else:
#                     color = (1,0,0);
#
#                 plane.position.xyz = x + 0.5,y + 0.5,0;
#                 plane.uniforms['diffuse'] = color;
#                 plane.uniforms['spec_weight'] = 0;
#                 grid.add_children(plane);
#
#         return grid;
#
#     def __init_entities(self):
#         self.obj_filename = rc.resources.obj_primitives;
#         self.obj_reader = rc.WavefrontReader(self.obj_filename);
#
#         self.quad_pool = EntityPool(self.__make_quadrotor);
#         self.axis_pool = EntityPool(self.__make_axis);
#         self.goal_pool = EntityPool(self.__make_goal);
#         self.grid_pool = EntityPool(self.__make_grid);
#
#         self.entity_pools = [self.quad_pool,
#                             self.axis_pool,
#                             self.goal_pool,
#                             self.grid_pool];
#
#     def __reset_drawing(self):
#         #Reset entity pools for entity re-use
#         [pool.reset() for pool in self.entity_pools];
#
#         # self.world.children = [];
#         [self.world.remove_children(x) for x in self.world.children];
#
#         grid_x = self.grid_pool.get();
#         grid_y = self.grid_pool.get();
#         grid_z = self.grid_pool.get();
#         grid_x_back = self.grid_pool.get();
#         grid_y_back = self.grid_pool.get();
#         grid_z_back = self.grid_pool.get();
#
#         grid_x.position.x = -3.0;
#         grid_y.position.y = -3.0;
#         grid_z.position.z = -3.0;
#         grid_x_back.position.x = 3.0;
#         grid_y_back.position.y = 3.0;
#         grid_z_back.position.z = 3.0;
#
#         grid_x.rotation.y = 90;
#         grid_y.rotation.x = -90;
#         grid_x_back.rotation.y = -90;
#         grid_y_back.rotation.x = 90;
#         grid_z_back.rotation.x = 180;
#
#         # grid_z.rotation.x = 90;
#
#         self.world.add_children(grid_x, grid_y, grid_z, grid_x_back, grid_y_back, grid_z_back);
#
#         self.labels = [];
#
#     def __init_window(self, width, height, name):
#         self.window = pyglet.window.Window(width=width, height=height, caption=name);
#         self.world = rc.EmptyEntity(name='world');
#         self.world.position.xyz = 0, 0, -2;
#         self.world.scale = 0.5;
#         self.grid = self.__make_grid();
#         self.scene = rc.Scene(meshes=self.world);
#         self.__reset_drawing();
#
#         @self.window.event
#         def on_draw():
#             with rc.default_shader:
#                 self.scene.draw();
#
#             [lbl.draw() for lbl in self.labels];
#             self.__reset_drawing();
#
#         @self.window.event
#         def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
#             rot_speed = 1.0;
#             self.world.rotation.y += dx * rot_speed;
#             self.world.rotation.x -= dy * rot_speed;
#
#
#         keys = key.KeyStateHandler()
#         self.window.push_handlers(keys)
#
#         def on_key_press(dt):
#             camera_speed = 3
#             # if keys[key.A]:
#             #     self.scene.camera.position.x -= camera_speed * dt
#             # if keys[key.D]:
#             #     self.scene.camera.position.x += camera_speed * dt
#             if keys[key.W]:
#                 self.scene.camera.position.z -= camera_speed * dt
#             if keys[key.S]:
#                 self.scene.camera.position.z += camera_speed * dt
#             if keys[key.R]:
#                 self.scene.camera.position.xyz = 0,0,0;
#                 self.world.rotation.xyz = 0,0,0;
#
#         pyglet.clock.schedule(on_key_press);
#
#
# class EntityPool:
#     def __init__(self, gen_func):
#         self.entities = [];
#         self.entitiesUsed = 0;
#         self.gen_func = gen_func;
#
#     def reset(self):
#         self.entitiesUsed = 0;
#
#     def get(self):
#         if(len(self.entities) <= self.entitiesUsed):
#             entity = self.gen_func();
#             self.entities.append(entity);
#         else:
#             entity = self.entities[self.entitiesUsed];
#
#         self.entitiesUsed += 1;
#         return entity;
