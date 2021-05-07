from klampt import WorldModel,RobotModel,RobotModelLink,Geometry3D
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from klampt import vis
from klampt.io import numpy_convert
import math
import numpy as np
from klampt.model.create import primitives
import sys
import time

from piano import Piano
from music_functions import KeyAction
import music_functions as mus
import arm

start_time = -1
motion_plan = None
robot = None

def play_piano(world, robot, piano, actions, height_offset):
    #keys, black_keys, plank = piano
    init_cf = robot.getConfig()
    traj = RobotTrajectory(robot, milestones=[init_cf], times=[0])

    for action in actions:
        #Form and execute motion plan

        #Fetch the keys being played
        keys = action.keys
        print(keys, len(keys), type(keys))

        #Get the config of the robot playing the chord
        played_cf = arm.play_chord(world, robot, piano, action, keys)
        if not played_cf:
            print("Unable to get play_chord config")
            return None

        #Get the finger transforms, and adjust the height by the offset
        robot.setConfig(played_cf)
        finger_links = arm.get_finger_links(robot)
        objectives = []
        for link in finger_links:
            #Position
            r_link = robot.link(link)
            ct = r_link.getWorldPosition(r_link.getTransform()[1])
            ft = (ct[0], ct[1], ct[2] + height_offset)
            print(r_link.getTransform()[1])
            print(ft)
            obj = ik.objective(r_link, local=r_link.getTransform()[1], world=ft)
            objectives.append(obj)

        played_keys = []
        for target in action.target_locs:
            played_keys.append(target)
        
        status = ik.solve_global(objectives, iters=100, feasibilityCheck=lambda : arm.is_collision_free_chord(world, robot, played_keys, piano))
        if not status:
            print("Unable to find successful configuration")
            return None
        ascend_cf = robot.getConfig()


        traj = traj.concat(RobotTrajectory(robot,milestones=[ascend_cf, played_cf, ascend_cf], times=[2.0, 4.0, 6.0]), relative=True, jumpPolicy="jump")
        break

    robot.setConfig(init_cf)
    return traj
    

def main(args):
    global motion_plan, robot
    #Validate args
    if len(args) != 2:
        print("Command must have 2 arguments")
        print("Format: python motionplan.py [midi filepath]")
        print("Exiting...")
        return -1

    #Load the track
    track = mus.load_midi(args[1],1)
    if not track:
        print("MIDI file did not load properly")
        print("Exiting...")
        return -1
    actions = mus.track_to_seq(track)

    #Instantiate robot model
    world = arm.load_model_world()
    vis.add("world", world)

    #Instantiate piano model
    piano_scale = 0.25
    location = [-0.5, 1.0, 0.2]
    piano_obj = Piano(world, piano_scale, location)
    piano = piano_obj.get_components()
    keys, black_keys, plank = piano
    vis.add('white_keys', keys)
    vis.add('black_keys', black_keys)
    vis.add('piano_body', plank)

    #Do robot stuff
    robot = world.robot(0)
    #robot.randomizeConfig()
    base_link = robot.link('ra_base_link')

    #print("Length of key actions")
    #print(len(actions))
    motion_plan = play_piano(world, robot, piano_obj, actions, 0.05)
    if not motion_plan:
        print("Plan failed")
        return -1
    vis.add('plan', motion_plan)

    def startPlan():
        global start_time
        start_time = time.time()
    vis.addAction(startPlan, 'Starting motion plan', 'r')

    def callback():
        global robot, motion_plan, start_time
        print("callback")
        if start_time == -1:
            return 
        ct = time.time() - start_time
        cf = motion_plan.eval(ct)
        robot.setConfig(cf)
        if ct > motion_plan.duration():
            start_time = -1

    vis.loop(callback=callback)


if __name__=='__main__':
    main(sys.argv)
    


    



    