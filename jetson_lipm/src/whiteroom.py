from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
import math
import numpy as np
import time


import imu 



from sklearn.preprocessing import normalize
from scipy.spatial.transform import Rotation as R

# from jax import grad,jacobian
# from pyassimp import load

from quaternion import Quaternion as qt
from quaternion import Haal


# from link_geometry import CoreCube

from bhram import Thing,Scene,bhram_vertex_shader,bhram_fragment_shader

# from bot import Manipulator as mp

from OpenGL.GL.shaders import compileShader,compileProgram

import shaders as shader


thread_len1 = 1;
thread_len2 = 1;
position = [-1000,0,1000]
old_position = []
active_pos = Haal()
active_pos.locate(*position)

sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)

bhram_program = 0


# Shader global variables
shaders_programID         = None
shaders_frustumID         = None
shaders_viewID            = None
shaders_template_location = None
shaders_position_location = None
shaders_color_location    = None

# VBO global variables
template_buffer  = None
position_buffer  = None
color_buffer     = None
template_data    = None
position_data    = None
color_data       = None
VERTEX_SIZE      = 3 # 3 vertices per triangle
POSITION_SIZE    = 3 # xyz
COLOR_SIZE       = 3 # rgb

mag = lambda v: math.sqrt(sum([i*i for i in v]))
nz = lambda v: [i/mag(v) for i in v]

class Room(object):
    def __init__(self,DRAW_SCENE):
        self.window_name = "Empty"
        self.camera_heading = [0,0,0]
        self.window_function()
        self.DRAW_SCENE = DRAW_SCENE

        self.angle = 0
        self.deltaAngle = 0
        self.alphaAngle = 0
        self.xOrigin = 0
        self.yOrigin = 0

    def update(self):
        self.draw_function()
    def look_at_scene(self):
        # glMatrixMode(GL_PROJECTION)
        global position,active_pos
        position = active_pos.position_P
        d = math.sqrt(sum([x*x for x in position]))
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40.,1.,d/100.,4*d)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(position[0],position[1],position[2],
          0,0,0,
          0,0,1)
    def mouseButton(self,button,state,x,y):
        global position,active_pos,old_position
        print(button,state,x,y)
        # // only start motion if the left button is pressed
        if button == 0 :
            # // when the button is released
            old_position = position
            if state == 1 :
                self.angle = self.angle + self.deltaAngle;
                self.xOrigin = -1;
            else:
                state = 0
                self.xOrigin = x;
                self.yOrigin = y;
        if button == 3 :
            position = [position[0]*(1-0.1),position[1]*(1-0.1),position[2]*(1-0.1)]
        if button == 4 :
            position = [position[0]*(1+0.1),position[1]*(1+0.1),position[2]*(1+0.1)]
        active_pos.locate(*position)
    def mouseMove(self,x,y):
        global position,old_position
        # print(x,y)
        # print(self.xOrigin)
        # // this will only be true when the left button is down
        if self.xOrigin >= 0 :
            # // update deltaAngle
            self.deltaAngle = (x - self.xOrigin)*10;
            self.alphaAngle = (y - self.yOrigin)*10;
            # print(self.deltaAngle,self.alphaAngle)
            # // update camera's direction
            # lx = sin(angle + deltaAngle);
            # lz = -cos(angle + deltaAngle);
            position = np.add(old_position,(0,0,self.alphaAngle))
            position = np.add(position,np.cross(nz(position),[0,0,1])*self.deltaAngle)
            theta = math.atan2(position[0],position[1]);
            # curr_perp = [0,theta,0]
            # gluLookAt(0,0,10,
            #           0,0,0,
            #           0,1,1)
        active_pos.locate(*position)

        # }
        # }
    def draw_room(self):
        glPushMatrix()
        # glTranslatef(position[0],position[1],position[2])
        # glutWireCube(1000.0)
        glPopMatrix()
    def keyboard_funtion(self,*args):
        global position,active_pos
        key = str(args[0],'utf-8')
        # print(key)
        if key == ' ':
            snap_zero = True
        if key == 'r':
            active_pos.locate(100,100,100)
    def draw_display(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        # glClearColor(1.,1.,1.,1.)
        color = [1.0,0.,0.,1.]
        self.look_at_scene()
        glPushMatrix()
        self.draw_room()
        self.DRAW_SCENE.make_scene()
        glPopMatrix()
        glutSwapBuffers()
        return
    def window_function(self):

        if(glutInit):
            glutInit(sys.argv)
        else :
            return
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(300,300)
        glutCreateWindow(self.window_name)

        glClearColor(0.3137254901960784,0.43137254901960786,0.7647058823529411,1.)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glutKeyboardFunc(self.keyboard_funtion)
        glutMouseFunc(self.mouseButton);
        glutMotionFunc(self.mouseMove);        
        glutDisplayFunc(self.draw_display)
        glMatrixMode(GL_PROJECTION)
        gluPerspective(40.,1.,1.,40.)
        glMatrixMode(GL_MODELVIEW)
        gluLookAt(0,0,10,
                  0,0,0,
                  0,1,1)
        glPushMatrix()
        # glutMainLoop()
        return
    def draw_function(self):
        glutPostRedisplay()
        glutMainLoopEvent()


# _CELESTIAL_SPHERE_ = Thing(glutWireSphere,(100,20,20))

SCENE_1 = Scene()

# SCENE_1.add_object(_CELESTIAL_SPHERE_)

SCENE_1.fix_position([0,0,0])


def make_goal_axis(a,b):
    glPushMatrix()
    glColor3f(1.0, 0., 0.);
    glScalef(b,a,a)
    glutSolidCube(10)
    glPopMatrix()

    glPushMatrix()        
    glColor3f(0., 1.0,0.);
    glScalef(a,b,a)
    glutSolidCube(10)
    glPopMatrix()

    glPushMatrix()
    glColor3f(0., 0., 1.0);
    glScalef(a,a,b)
    glutSolidCube(10)
    glPopMatrix()  
    

    glColor3f(1., 1., 1.);

_STATE_ = Thing(make_goal_axis,(2,10))
_STATE_ACC = Thing(make_goal_axis,(1,32))
SCENE_1.add_object(_STATE_)
SCENE_1.add_object(_STATE_ACC)



SCENE_MAIN = Scene()

SCENE_MAIN.add_scene(SCENE_1,1,1,1)

R_viz = Room(SCENE_MAIN)


# from_MATRIX
IMU = imu.Accelerometer()
g_int = [0,0,0]
g_drift = [0,0,0]

a_avg = [[0,0,0]];
a_len = 25;


time_old = time.time()
def degrees(v):
    return [k*180/math.pi for k in v]


cal_steps = 100;
done_steps = 0;

print("calibration")
while done_steps <= cal_steps:
    done_steps += 1;
    time_now = time.time()
    accs = list(IMU.readAccData());
    gyros = list(IMU.readGyroData());

    # # a_avg.pop(0)
    # a_avg.append(accs)
    # # if len(a_avg) > a_len:
    # a_avg = a_avg[:-a_len]


    time_delta = time_now - time_old;
    g_drift = [g_drift[l] + gyros[l]*time_delta for l in range(len(gyros))];
    time_old = time.time()

g_drift = [g/cal_steps for g in g_drift]
print("done calibration")
print(g_drift)


print("staring ICP walk")


frame = 0;
reset_clock = 100;
alpha = 0
beta = 0

def avg(matrix):
    vals = [];
    n = len(matrix);
    if n <= 0:
        return matrix
    m = len(matrix[0]);

    for k in range(m):
        vals.append(0)
    for v in matrix:
        for k in range(m):
            vals[k] += v[k]
    for k in range(m):
        vals[k] /= n
    return vals
while 21:
    frame += 1;

    time_now = time.time()
    accs = list(IMU.readAccData());
    gyros = list(IMU.readGyroData());

    time_delta = (time_now - time_old)*25;
    g_int = [g_int[l] + (gyros[l] - g_drift[l])*time_delta for l in range(len(gyros))];
    time_old = time.time()

    # a_avg.pop(0)
    a_avg.append(accs)
    # if len(a_avg) > a_len:
    a_avg = a_avg[-a_len:]
    
    accs_av = avg(a_avg);

    alpha = math.atan2(accs_av[0],accs_av[2])
    beta = -math.atan2(accs_av[1],accs_av[2])


    if frame % reset_clock == 0:
        g_int = [ beta , alpha, 0 ]


    # g_int = degrees()
    # print(time_delta,(degrees(g_int)))
    gm = R.from_euler('zyx',g_int).as_quat();
    am = R.from_euler('xzy',[ 0 ,beta ,alpha]).as_quat();
    # gm = R.from_euler('zyx',degrees(g_int)).as_quat();

    # _STATE_.haal.rotation_Q = qt.from_value([gm[3],gm[0],gm[1],gm[2]]);
    _STATE_.haal.rotation_Q = qt.from_value(gm);
    _STATE_ACC.haal.rotation_Q = qt.from_value(am);
    # _STATE_.haal.rotation_Q = qt.from_MATRIX(gm);
    # _STATE_.haal.rotation_Q = qt.from_value(gm);
    

    R_viz.update()

# #     for i in range(len(joint_values)):
# #         joint_values[i] += thread_len1*10
# #     # joint_values = [
# #     #     thread_len1*5,
# #     #     thread_len2*5,
# #     #     0
# #     # ]

# #     my_manip.set_joint_angles(joint_values);
#     # time.sleep(1/24)
#     for i in range(1000000):
#         pass
