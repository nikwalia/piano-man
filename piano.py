from klampt import WorldModel,RobotModel,RobotModelLink,Geometry3D
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from klampt import vis
import math
import numpy as np

from klampt.model.create import primitives

#I seem to have reached an impasse; that importing the robot model into klampt requires it to be compiled without edits,
#meaning it has to successfully load and I cannot modify it, and unfortunately enough the Shadowhand has empty triangles
#(2 sides are the same point) so it won't load the model, let alone whether or not it will work once fixed. Guess I'll ask Kris
#this week?


#you will need to change this to the absolute or relative path to Klampt-examples
KLAMPT_EXAMPLES = '../cs498ir_s2021'
#KLAMPT_EXAMPLES = '../../../cs498ir_s2021'

# def make_shelf(world,width,depth,height,wall_thickness=0.005):
#     """Makes a new axis-aligned "shelf" centered at the origin with
#     dimensions width x depth x height. Walls have thickness wall_thickness.
#     """
#     left = Geometry3D()
#     right = Geometry3D()
#     back = Geometry3D()
#     bottom = Geometry3D()
#     top = Geometry3D()
#     #method 1
#     left.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
#     left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
#     right.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
#     right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
#     #method 2
#     back.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
#     back.scale(width,wall_thickness,height)
#     back.translate([-width*0.5,depth*0.5,0])
#     #equivalent to back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
#     #method 3
#     bottom = primitives.box(width,depth,wall_thickness,center=[0,0,0])
#     top = primitives.box(width,depth,wall_thickness,center=[0,0,height-wall_thickness*0.5])
#     shelfgeom = Geometry3D()
#     shelfgeom.setGroup()
#     for i,elem in enumerate([left,right,back,bottom,top]):
#         g = Geometry3D(elem)
#         shelfgeom.setElement(i,g)
#     shelf = world.makeTerrain("shelf")
#     shelf.geometry().set(shelfgeom)
#     shelf.appearance().setColor(0.2,0.6,0.3,1.0)
#     return shelf

#52 white keys, 36 black keys
#Base scale is width of piano key = 0.1 at scale 1.0
def make_piano(world,scale):
    key_width = 0.11 * scale
    key_height = 0.1 * scale
    black_key_width = 0.05 * scale
    keys = []
    black_keys = []
    for i in range(52):
        key = make_key(scale)
        key.translate([key_width * i,0,0.5 * scale])
        black_key = make_black_key(scale, i)
        if black_key:
            black_key.translate([key_width * i - 1/2 * key_width + 1/2 * black_key_width,0,(key_height + 0.55) * scale])
            black_keys.append(black_key)
        keys.append(key)
        
    keyG = make_component(world, "white_keys",keys,(1,1,1))
    black_keyG = make_component(world, "black_keys",black_keys,(0,0,0))
    return keyG, black_keyG

def make_component(world, name, g, color):
    model = Geometry3D()
    model.setGroup()
    for i,elem in enumerate(g):
        t = Geometry3D(elem)
        model.setElement(i,t)
    modelG = world.makeTerrain(name)
    modelG.geometry().set(model)
    modelG.appearance().setColor(color[0], color[1], color[2])
    return modelG

def make_key(scale):
    key_width = 0.1 * scale
    key_height = 0.1 * scale
    key_length = 0.6 * scale
    key = Geometry3D()
    key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
    key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
    return key

def make_black_key(scale, index):
    if index % 7 == 0 or index % 7 == 3:
        return None
    key_width = 0.05 * scale
    key_height = 0.05 * scale
    key_length = 0.6 * 0.6 * scale
    key = Geometry3D()
    key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
    key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
    return key

def create_terrain(world, width, length):
    geo = primitives.box(width,length,0.05,center=[0,0,0])
    terrain = world.makeTerrain("ground")
    terrain.geometry().set(geo)
    return terrain

def show_env():
    world = WorldModel()
    # terrain = create_terrain(world, 5, 5)
    #file="sr_common-melodic-devel/sr_description/mujoco_models/urdfs/shadowhand_motor.urdf"
    # robot = RobotModel()
    # robot = robot.loadFile("sr_common-melodic-devel/sr_description/mujoco_models/urdfs/shadowhand_motor")
    status = world.readFile("world.xml")
    if not status:
        print("file not read")
        exit(-1)
    vis.add("world", world)
    #shelf = make_shelf(world, 0.5, 0.5, 0.5)
    piano_scale = 0.25
    # keys, black_keys = make_piano(world, piano_scale)
    # vis.add("white_keys", keys)
    # vis.add("black_keys", black_keys)
    # cam = vis.camera.free()

    link_names = ['ra_base_link', 'ra_shoulder_link', 'ra_upper_arm_link',\
                    'ra_forearm_link', 'ra_wrist_1_link', 'ra_wrist_2_link', 'ra_wrist_3_link']

    robot = world.robot(0)
    flipyz = so3.rotation([1,0,0], math.pi / 2)
    for i in range(robot.numLinks()):
        if robot.link(i).getName() in link_names:
            robot.link(i).geometry().transform(flipyz,[0,0,0])

    vis.run()

if __name__=='__main__':
    show_env()
