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
        self.plank = Geometry3D()
        self.plank.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        self.plank.transform([total_width,0,0,0,length,0,0,0,depth],
        [self.location[0] + (0),
        self.location[1] + (-1 * length),
        self.location[2] + (0.30 * self.scale)])
        modelG = self.world.makeTerrain(name)
        modelG.geometry().set(self.plank)
        modelG.appearance().setColor(*color)
        return modelG


    def get_key_target(self, key_id: int):
        """
        Gets the actual physical target coordinates in Klampt-space.
        """
        if key_id in self.piano_definition['white']:
            key = self.piano_definition['white'][key_id]
            bb = key.getBBTight()
            # 3/4 length of key
            xl = abs(bb[0][0] - bb[1][0])
            yl = abs(bb[0][1] - bb[1][1])

            x = min(bb[0][0], bb[1][0]) + (3 * xl / 4)
            y = min(bb[0][1], bb[1][1]) + (3 * yl / 4)
            
        else:
            key = self.piano_definition['black'][key_id]
            bb = key.getBBTight()

            # middle of key
            xl = abs(bb[0][0] - bb[1][0])
            yl = abs(bb[0][1] - bb[1][1])

            x = min(bb[0][0], bb[1][0]) + (1 * xl / 2)
            y = min(bb[0][1], bb[1][1]) + (1 * yl / 2)

        z = max(bb[0][2], bb[1][2])

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

if __name__=='__main__':
    world = WorldModel()
    piano_scale = 0.25
    location = [0, 0, 0]
    keys, black_keys, plank = Piano(world, piano_scale, location).get_components()

    vis.add('white_keys', keys)
    vis.add('black_keys', black_keys)
    vis.add('piano_body', plank)

    vis.run()
    