from klampt import WorldModel,RobotModel,RobotModelLink,Geometry3D
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from klampt import vis
import math
import numpy as np

from klampt.model.create import primitives


#you will need to change this to the absolute or relative path to Klampt-examples
KLAMPT_EXAMPLES = '../cs498ir_s2021'
# KLAMPT_EXAMPLES = '../../../cs498ir_s2021'
WHITE_MIDI_KEYS = [21, 23, 24, 26, 28, 29, 31, 33, 35, 36, 38, 40, 41, 43, 45, 47, 48, 50, 52, 53, 55, 57, 59, 60, 62, 64, 65, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 84, 86, 88, 89, 91, 93, 95, 96, 98, 100, 101, 103, 105, 107, 108]
BLACK_MIDI_KEYS = [22, 25, 27, 30, 32, 34, 37, 39, 42, 44, 46, 49, 51, 54, 56, 58, 61, 63, 66, 68, 70, 73, 75, 78, 80, 82, 85, 87, 90, 92, 94, 97, 99, 102, 104, 106]


class Piano(object):
    #52 white keys, 36 black keys
    #Base scale is width of piano key = 0.1 at scale 1.0
    def __init__(self, world, scale):
        self.world = world

        self.key_width = 0.11 * scale
        self.key_height = 0.1 * scale
        self.black_key_width = 0.05 * scale
        self.key_length = 0.6 * scale
        self.black_key_length = 0.6 * 0.6 * scale
        self.scale = scale

        self.piano_definition = {'white': {}, 'black': {}}

        black_idx = 0

        for i in range(52):
            key = self.make_key()
            key.translate([self.key_width * i, -1 * self.key_length, 0.5 * self.scale])
            self.piano_definition['white'][WHITE_MIDI_KEYS[i]] = Geometry3D(key)

            black_key = self.make_black_key(i)
            if black_key:
                black_key.translate([self.key_width * i - 1/2 * self.key_width + 1/2 * self.black_key_width, -1 * self.black_key_length, (self.key_height + 0.55) * self.scale])
                self.piano_definition['black'][BLACK_MIDI_KEYS[black_idx]] = Geometry3D(black_key)
                black_idx += 1


    def get_components(self):
        keyG = self.make_component("white_keys", self.piano_definition['white'].values, (1, 1, 1))
        black_keyG = make_component("black_keys", self.piano_definition['black'].values, (0, 0, 0))

        return keyG, black_keyG

    def make_component(self, name: str, g, color: list):
        model = Geometry3D()
        model.setGroup()
        for i, elem in enumerate(g):
            # t = Geometry3D(elem)
            model.setElement(i, elem)
        modelG = self.world.makeTerrain(name)
        modelG.geometry().set(model)
        modelG.appearance().setColor(*color)
        return modelG

    def make_key(self):
        key_width = 0.1 * self.scale
        key_height = 0.1 * self.scale
        key_length = 0.6 * self.scale
        key = Geometry3D()
        key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
        return key

    def make_black_key(self, index):
        if index % 7 == 2 or index % 7 == 5 or index == 0 or index == 51:
            return None
        key_width = 0.05 * self.scale
        key_height = 0.05 * self.scale
        key_length = 0.6 * 0.6 * self.scale
        key = Geometry3D()
        key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
        return key

    def get_key_target(self, key_id: int):
        if key_id in self.piano_definition['white']:
            key = self.piano_definition['white'][key_id]
            bb = key.getBBTight()

            # 3/4 length of key
            x = 3 * (bb[0][0] + bb[1][0]) / 4
            y = 3 * (bb[0][1] + bb[1][1]) / 4
            
        else:
            key = self.piano_definition['black'][key_id]
            bb = key.getBBTight()

            # middle of key
            x = (bb[0][0] + bb[1][0]) / 2
            y = (bb[0][1] + bb[1][1]) / 2

        z = max(bb[0][2], bb[1][2])

        return (x, y, z)

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
    piano = Piano(world, piano_scale)
    keys, black_keys = piano.get_components()
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
    #print(chord)
    locations = []
        
    keysToPlay = getKeysForChord(chord)
    for key in keysToPlay:
        if key > 51:
            #Black key
            keyG = black_keys.geometry().getElement(key - 52)
            bb = keyG.getBBTight()
            z = max(bb[0][2], bb[1][2])
            #Middle of key
            x = (bb[0][0] + bb[1][0]) / 2
            y = (bb[0][1] + bb[1][1]) / 2
            locations.append((x,y,z))
        else:
            #White key
            keyG = white_keys.geometry().getElement(key)
            bb = keyG.getBBTight()
            z = max(bb[0][2], bb[1][2])
            #3/4 length of key
            x = 3 * (bb[0][0] + bb[1][0]) / 4
            y = 3 * (bb[0][1] + bb[1][1]) / 4
            locations.append((x,y,z))
        
    return locations

#Hard-coded table or other computation to retrieve key indices
#For black keys, offset by 52 to distinguish the values
def getKeysForChord(chord):
    return None
    
#Given the finger positions, play the chord
#three-note chords are assumed for now
#obj = ik.objective(robotmodellink,local=lclpt,world=wldpt)
#Fingers will be a list of the links for the fingers
def playChord(world, robot, fingers, locations, white_keys, black_keys, num_attempts, height_offset):
    #Get the initial robot config
#     init_cf = robot.getConfig()
#     attempt_count = 0
#     #Need to find a way to calculate finger coords
#     solver = ik.IKSolver(robot)
#     for i in range(len(locations)):
#         obj = ik.objective(fingers[i],local=finger_coords[i],world=locations[i])
#         solver.add(obj1)
#     config = solver.solve()
#     while attempt_count < num_attempts and (not config or (config and not not_colliding(world, robot, white_keys, black_keys))):
#         config = solver.solve()
#         if not config:
#             print("valid position not found")
    
#     final_cf = robot.getConfig()
    def not_colliding_adj():
        return not_colliding(world, robot, white_keys, black_keys)
    
    #Construct the motion plan for moving, descending, and ascending (move the collision checking here)
    #Will likely need to set activedofs later
    init_cf = robot.getConfig()
    finger_links = getFingerLinks(robot)
    finger_tfs = []
    for finger in finger_links:
        finger_tfs.append(finger.getTransform())
    
    objs = []
    for i in range(len(locations)):
        obj = ik.objective(fingers[i],local=finger_coords[i],world=locations[i])
        objs.append(obj)
    status = ik.solve_global(objectives, iters=1000, numRestarts=100, feasibilityCheck=not_colliding_adj)
    if not status:
        print("Could not find a solution")
        return None
    final_cf = robot.getConfig()
    
    #Now get the higher coordinates using height_offset
    
    finger_tfs_adj = []
    for i in range(len(locations)):
        finger_links[i].setTransform((finger_tfs[i][0], (finger_tfs[i][1][0], finger_tfs[i][1][1], finger_tfs[i][1][2] + height_offset)))
    above_cf = robot.getConfig()
    
    return RobotTrajectory(robot,milestones=[init_cf, above_cf, final_cf, above_cf])
    
        
                                   
        
#Utility function, returns a list of the robot links containing the fingers
def getFingerLinks(robot):
    return None
    
#To simplify some possible smaller issues, collisions between the robot and the keys won't be checked
def not_colliding(world, robot, white_keys, black_keys):
    if robot.selfCollides():
        return False
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
