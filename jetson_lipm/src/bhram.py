from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from OpenGL.GL import shaders

from quaternion import Quaternion as qt
from quaternion import Haal

import math
import textwrap

#Engine 3.24.2.23

class Thing(object):
    def __init__(self,draw_function,draw_args):
        self.draw_function = draw_function
        self.draw_args = draw_args
        self.haal = Haal()
        self.color = (1,1,1)
    def make(self):
        # Q_filterred = self.Q_filterred
        w, v = self.haal.rotation_Q.get_axisangle()
        w_degrees = w*180/math.pi
        location = self.haal.position_P
        glPushMatrix()
        glColor3f(*self.color);
        glTranslatef(location[0],location[1],location[2])
        # print(location)
        glRotatef(w_degrees,v[0],v[1],v[2])
        self.draw_function(*self.draw_args)
        glPopMatrix()
    def locate(self,x,y,z):
        self.haal.locate(x,y,z)

class Scene(object):
    def __init__(self):
        self.objects = []
        self.scenes = []
        self.scales = []
        self.coordinates = [0.,0.,0.]
        self.haal = Haal()
    def add_object(self,thing):
        self.objects.append(thing)
    def add_objects(self,things):
        [self.objects.append(thing) for thing in things]
    def add_scene(self,scene,x,y,z):
        self.scenes.append(scene)
        self.scales.append([x,y,z])
    def fix_position(self,coords):
        self.coordinates = coords
    def make_scene(self,depth=0):
        if depth > 10:
            return
        w, v = self.haal.rotation_Q.get_axisangle()
        w_degrees = w*180/math.pi
        glPushMatrix()
        glTranslatef(self.coordinates[0],self.coordinates[1],self.coordinates[2])
        glRotatef(w_degrees,v[0],v[1],v[2])
        for o in self.objects:
            # print(o)
            o.make()
        for s in range(len(self.scenes)):
            scale = self.scales[s]
            glPushMatrix()
            glScalef(scale[0],scale[1],scale[2])
            self.scenes[s].make_scene(depth+1)
            glPopMatrix()
        glPopMatrix()


# bhram_vertex_shader ="""
# varying vec3 vN;
# varying vec3 v;
# varying vec4 color;
# void main(void)  
# {     
#    v = vec3(gl_ModelViewMatrix * gl_Vertex);       
#    vN = normalize(gl_NormalMatrix * gl_Normal);
#    color = gl_Color;
#    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;  
# }
# """


# bhram_fragment_shader ="""
#     varying vec3 vN;
#     varying vec3 v; 
#     varying vec4 color;
#     #define MAX_LIGHTS 1 
#     void main (void) 
#     { 
#        vec3 N = normalize(vN);
#        vec4 finalColor = vec4(0.0, 0.0, 0.0, 0.0);
       
#        for (int i=0;i<MAX_LIGHTS;i++)
#        {
#           vec3 L = normalize(gl_LightSource[i].position.xyz - v); 
#           vec3 E = normalize(-v); // we are in Eye Coordinates, so EyePos is (0,0,0) 
#           vec3 R = normalize(-reflect(L,N)); 
       
#           vec4 Iamb = gl_LightSource[i].ambient; 
#           vec4 Idiff = gl_LightSource[i].diffuse * max(dot(N,L), 0.0);
#           Idiff = clamp(Idiff, 0.0, 1.0); 
#           vec4 Ispec = gl_LightSource[i].specular * pow(max(dot(R,E),0.0),0.3*gl_FrontMaterial.shininess);
#           Ispec = clamp(Ispec, 0.0, 1.0); 
       
#           finalColor += Iamb + Idiff + Ispec;
#        }
#        gl_FragColor = color * finalColor; 
#     }
#     """

bhram_vertex_shader = textwrap.dedent( """
#version 150

// input
in vec3 in_position;
in vec3 in_colour;
uniform mat4 model_view;
uniform mat4 projection;

// shared
out vec3 ex_colour;

void main(void) 
{
    // apply projection and model view matrix to vertex
    gl_Position = projection * model_view * vec4( in_position, 1.0 );

    ex_colour = in_colour;
}
""" )

bhram_fragment_shader = textwrap.dedent( """
#version 150

// shared
in vec3 ex_colour;

// output
out vec4 fragColor;

void main(void) 
{
    // set colour of each fragment
    fragColor = vec4( ex_colour, 1.0 );
}
""" )
