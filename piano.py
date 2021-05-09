from klampt import WorldModel,RobotModel,RobotModelLink,Geometry3D
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from klampt import vis
from klampt.io import numpy_convert
import math
import numpy as np

from klampt.model.create import primitives


#you will need to change this to the absolute or relative path to Klampt-examples
KLAMPT_EXAMPLES = '../cs498ir_s2021'
# KLAMPT_EXAMPLES = '../../../cs498ir_s2021'
WHITE_MIDI_KEYS = [21, 23, 24, 26, 28, 29, 31, 33, 35, 36, 38, 40, 41, 43, 45, 47, 48, 50, 52, 53, 55,
                    57, 59, 60, 62, 64, 65, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 84, 86, 88, 89, 91,
                    93, 95, 96, 98, 100, 101, 103, 105, 107, 108]
BLACK_MIDI_KEYS = [22, 25, 27, 30, 32, 34, 37, 39, 42, 44, 46, 49, 51, 54, 56, 58, 61, 63, 66, 68, 70,
                    73, 75, 78, 80, 82, 85, 87, 90, 92, 94, 97, 99, 102, 104, 106]


class Piano(object):
    """
    Represents and creates a piano with helper methods for accessing data
    
    52 white keys, 36 black keys
    Base scale is width of piano key = 0.1 at scale 1.0
    """
    
    def __init__(self, world, scale, location):
        self.world = world

        self.location = location
        self.key_width = 0.11 * scale
        self.key_height = 0.1 * scale
        self.black_key_width = 0.05 * scale
        self.key_length = 0.6 * scale
        self.black_key_length = 0.6 * 0.6 * scale
        self.scale = scale

        self.piano_definition = {'white': {}, 'black': {}}

        black_idx = 0

        # TODO create plank underneath. this can be used to make sure the key is not being played from underneath

        for i in range(52):
            key = self.make_key()
            key.translate([location[0] + (self.key_width * i), 
                            location[1] + (-1 * self.key_length), 
                            location[2] + (0.5 * self.scale)])
            self.piano_definition['white'][WHITE_MIDI_KEYS[i]] = Geometry3D(key)

            black_key = self.make_black_key(i)
            if black_key:
                black_key.translate([location[0] + (self.key_width * i - 1/2 * self.key_width + 1/2 * self.black_key_width),
                                    location[1] + (-1 * self.black_key_length),
                                    location[2] + ((self.key_height + 0.55) * self.scale)])
                self.piano_definition['black'][BLACK_MIDI_KEYS[black_idx]] = Geometry3D(black_key)
                black_idx += 1


    def get_components(self):
        """
        Used for visualization
        """
        keyG = self.make_component("white_keys", self.piano_definition['white'].values(), (1, 1, 1))
        black_keyG = self.make_component("black_keys", self.piano_definition['black'].values(), (0, 0, 0))
        plank = self.make_piano_body("piano_body", (89/255, 29/255, 17/255))

        return keyG, black_keyG, plank

    def make_component(self, name: str, g, color: list):
        """
        Creates components for visualization/addition to Klampt
        """
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
        """
        Creates a white key
        """
        key_width = 0.1 * self.scale
        key_height = 0.1 * self.scale
        key_length = 0.6 * self.scale
        key = Geometry3D()
        key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
        return key

    def make_black_key(self, index):
        """
        Creates a black key. May return None if there shouldn't be a black key according to grand
        piano config
        """
        if index % 7 == 2 or index % 7 == 5 or index == 0 or index == 51:
            return None
        key_width = 0.05 * self.scale
        key_height = 0.05 * self.scale
        key_length = 0.6 * 0.6 * self.scale
        key = Geometry3D()
        key.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        key.transform([key_width,0,0,0,key_length,0,0,0,key_height],[0, 0, 0])
        return key

    def make_piano_body(self, name, color: list):
        scale = self.scale
        total_width = 0.1 * self.scale * 52 + 0.01 * self.scale * 51
        length = 0.6 * self.scale
        depth = 0.15 * self.scale
        plank = Geometry3D()
        plank.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        plank.transform([total_width,0,0,0,length,0,0,0,depth],
        [self.location[0] + (0),
        self.location[1] + (-1 * length),
        self.location[2] + (0.30 * self.scale)])
        modelG = self.world.makeTerrain(name)
        modelG.geometry().set(plank)
        modelG.appearance().setColor(*color)
        return modelG


    def get_key_target(self, key_id: int):
        """
        Gets the actual physical target coordinates in Klampt-space.
        """
        print("KEYID", key_id)
        if key_id in self.piano_definition['white']:
            print("White key played")
            key = self.piano_definition['white'][key_id]
            bb = key.getBBTight()
            print(bb)
            #print('bb0', bb[0], 'bb1', bb[1])
            # 3/4 length of key
            xl = abs(bb[0][0] - bb[1][0])
            yl = abs(bb[0][1] - bb[1][1])

            x = min(bb[0][0], bb[1][0]) + (3 * xl / 4)
            y = min(bb[0][1], bb[1][1]) + (3 * yl / 4)
            
        else:
            print("Black key played")
            key = self.piano_definition['black'][key_id]
            bb = key.getBBTight()
            #print('bb0', bb[0], 'bb1', bb[1])

            # middle of key
            xl = abs(bb[0][0] - bb[1][0])
            yl = abs(bb[0][1] - bb[1][1])

            x = min(bb[0][0], bb[1][0]) + (1 * xl / 2)
            y = min(bb[0][1], bb[1][1]) + (1 * yl / 2)

        z = max(bb[0][2], bb[1][2])

        print(x,y,z)
        return (x, y, z)

def create_terrain(world, width, length):
    geo = primitives.box(width,length,0.05,center=[0,0,0])
    terrain = world.makeTerrain("ground")
    terrain.geometry().set(geo)
    return terrain

#Motion planning
#1 - Robot translates chord into key locations
#2 - Robot moves to location above keys
#3 - Robot descends to keys
#4 - Robot lifts hand back up
#5 - Repeat 1-4 for remaining keys

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
    world = WorldModel()
    piano_scale = 0.25
    location = [0, 0, 0]
    keys, black_keys, plank = Piano(world, piano_scale, location).get_components()

    vis.add('white_keys', keys)
    vis.add('black_keys', black_keys)
    vis.add('piano_body', plank)

    vis.run()
    