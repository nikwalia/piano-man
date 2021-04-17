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

def make_shelf(world,width,depth,height,wall_thickness=0.005):
    """Makes a new axis-aligned "shelf" centered at the origin with
    dimensions width x depth x height. Walls have thickness wall_thickness.
    """
    left = Geometry3D()
    right = Geometry3D()
    back = Geometry3D()
    bottom = Geometry3D()
    top = Geometry3D()
    #method 1
    left.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
    left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
    right.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
    right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
    #method 2
    back.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
    back.scale(width,wall_thickness,height)
    back.translate([-width*0.5,depth*0.5,0])
    #equivalent to back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
    #method 3
    bottom = primitives.box(width,depth,wall_thickness,center=[0,0,0])
    top = primitives.box(width,depth,wall_thickness,center=[0,0,height-wall_thickness*0.5])
    shelfgeom = Geometry3D()
    shelfgeom.setGroup()
    for i,elem in enumerate([left,right,back,bottom,top]):
        g = Geometry3D(elem)
        shelfgeom.setElement(i,g)
    shelf = world.makeTerrain("shelf")
    shelf.geometry().set(shelfgeom)
    shelf.appearance().setColor(0.2,0.6,0.3,1.0)
    return shelf

def create_terrain(world, width, length):
    geo = primitives.box(width,length,0.05,center=[0,0,0])
    terrain = world.makeTerrain("ground")
    terrain.geometry().set(geo)
    return terrain

def show_env():
    print("something")
    world = WorldModel()
    terrain = create_terrain(world, 5, 5)
    #file="sr_common-melodic-devel/sr_description/mujoco_models/urdfs/shadowhand_motor.urdf"
    # robot = RobotModel()
    # robot = robot.loadFile("sr_common-melodic-devel/sr_description/mujoco_models/urdfs/shadowhand_motor")
    status = world.readFile("world.xml")
    if not status:
        print("file not read")
        exit(-1)
    vis.add("world", world)
    #shelf = make_shelf(world, 0.5, 0.5, 0.5)
    
    # cam = vis.camera.free()
    vis.run()

if __name__=='__main__':
    show_env()
