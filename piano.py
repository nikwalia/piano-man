from klampt import WorldModel,RobotModel,RobotModelLink,Geometry3D
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from klampt import vis
import math
import numpy as np

from klampt.model.create import primitives


#you will need to change this to the absolute or relative path to Klampt-examples
#KLAMPT_EXAMPLES = '../cs498ir_s2021'
KLAMPT_EXAMPLES = '../../../cs498ir_s2021'


#52 white keys, 36 black keys
#Base scale is width of piano key = 0.1 at scale 1.0
def make_piano(world,scale):
    key_width = 0.11 * scale
    key_height = 0.1 * scale
    black_key_width = 0.05 * scale
    key_length = 0.6 * scale
    black_key_length = 0.6 * 0.6 * scale
    keys = []
    black_keys = []
    for i in range(52):
        key = make_key(scale)
        key.translate([key_width * i,-1 * key_length,0.5 * scale])
        black_key = make_black_key(scale, i)
        if black_key:
            black_key.translate([key_width * i - 1/2 * key_width + 1/2 * black_key_width,-1 * black_key_length,(key_height + 0.55) * scale])
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
    if index % 7 == 2 or index % 7 == 5 or index == 0 or index == 51:
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
    keys, black_keys = make_piano(world, piano_scale)
    vis.add("white_keys", keys)
    vis.add("black_keys", black_keys)
    cam = vis.camera.free()
    locations = chordToKeys(world, "A", keys, black_keys)
    location = locations[0]
    vis.add("key",(location[0], location[1], location[2]))
#     link_names = ['ra_base_link', 'ra_shoulder_link', 'ra_upper_arm_link',\
#                     'ra_forearm_link', 'ra_wrist_1_link', 'ra_wrist_2_link', 'ra_wrist_3_link']

#     robot = world.robot(0)
#     flipyz = so3.rotation([1,0,0], math.pi / 2)
#     for i in range(robot.numLinks()):
#         if robot.link(i).getName() in link_names:
#             robot.link(i).geometry().transform(flipyz,[0,0,0])

    
    vis.run()
    
#Motion planning
#1 - Robot translates chord into key locations
#2 - Robot moves to location above keys
#3 - Robot descends to keys
#4 - Robot lifts hand back up
#5 - Repeat 1-4 for remaining keys

def strToChords(chords):
    #Loop through each character in the chord string
    for letter in chords:
        #Get the contact locations of the keys
        locations = chordToKeys(letter)
        #Skip invalid characters
        if locations == None:
            continue
            

#Returns the positions for the fingers to play the keys
def chordToKeys(world, chord, white_keys, black_keys):
    print(white_keys.geometry().numElements())
    print(black_keys.geometry().numElements())
    print(chord)
    locations = []
    for i in range(white_keys.geometry().numElements()):
        key = white_keys.geometry().getElement(i)
        bb = key.getBBTight()
        print(bb)
        #For white keys the contact point should be closer to the end
        #Height
        z = max(bb[0][2], bb[1][2])
        #Middle of key
        x = 3 * (bb[0][0] + bb[1][0]) / 4
        y = 3 * (bb[0][1] + bb[1][1]) / 4
        locations.append((x,y,z))
        break
        
    return locations
    
#Given the finger positions, play the chord
#three-note chords are assumed for now
#obj = ik.objective(robotmodellink,local=lclpt,world=wldpt)
#Fingers will be a list of the links for the fingers
def playChord(world, robot, fingers, locations, white_keys, black_keys, num_attempts):
    #Get the initial robot config
    init_cf = robot.getConfig()
    attempt_count = 0
    #Need to find a way to calculate finger coords
    solver = ik.IKSolver(robot)
    for i in range(len(locations)):
        obj = ik.objective(fingers[i],local=finger_coords[i],world=locations[i])
        solver.add(obj1)
    config = solver.solve()
    while attempt_count < num_attempts and (not config or (config and not not_colliding(world, robot, white_keys, black_keys))):
        config = solver.solve()
        if not config:
            print("valid position not found")
    
    final_cf = robot.getConfig()
    
    #Construct the motion plan for moving, descending, and ascending (move the collision checking here)
                                            
                                   
        
    
#To simplify some possible smalelr issues, collisions between the robot and the keys won't be checked
def not_colliding(world, robot, white_keys, black_keys):
    for i in range(robot.numLinks()):
        for j in range(world.numTerrains()):
            if robot.link(i).geometry().collides(world.terrain(i).geometry()) and world.terrain(i) != keys and world.terrain(i) != black_keys:
                return False
        for j in range(world.numRigidObjects()):
            if robot.link(i).geometry().collides(world.rigidObject(i).geometry()):
                return False
    return True
    
    

if __name__=='__main__':
    show_env()
