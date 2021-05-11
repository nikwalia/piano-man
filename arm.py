from klampt import WorldModel, RobotModel, RobotModelLink
from klampt import vis
from klampt.model import ik
from klampt.math import vectorops,so3,se3

from piano import Piano
from music_functions import KeyAction

ARM_LINK_NAMES = ['ra_base_link', 'ra_shoulder_link', 'ra_upper_arm_link',\
            'ra_forearm_link', 'ra_wrist_1_link', 'ra_wrist_2_link', 'ra_wrist_3_link']

FINGERTIP_LINK_NAMES = ['rh_thdistal', 'rh_ffdistal', 'rh_mfdistal', 'rh_rfdistal', 'rh_lfdistal']


def get_finger_links(robot: RobotModel):
    links = {key: None for key in FINGERTIP_LINK_NAMES}
    for i in range(robot.numLinks()):
        if robot.link(i).getName() in FINGERTIP_LINK_NAMES:
            links[robot.link(i).getName()] = robot.link(i)
    
    return links

def get_fingertip(link: RobotModelLink):
    bb_min, bb_max = link.geometry().getBBTight()
    finger_len = -1
    for i in range(3):
        if abs(bb_max[i] - bb_min[i]) > finger_len:
            finger_len = abs(bb_max[i] - bb_min[i])
    
    pos = link.getTransform()[1]
    com = link.getWorldPosition(link.getMass().getCom())
    axis = vectorops.unit(vectorops.sub(com, pos))
    print("pos", pos)
    print("com", com)
    print("final", vectorops.madd(pos, axis, finger_len))
    return vectorops.madd(pos, axis, finger_len)

def numFingers(locs):
    l = []
    for loc in locs:
        if locs[loc] != -1:
            l.append(loc)
    return len(l)

def play_chord(world: WorldModel, robot: RobotModel, piano: Piano, action: KeyAction, playing_keys: list, height_offset):
    action.convert_targets(piano)
    played_keys = []
    objectives = []
    print("fingers used", numFingers(action.target_locs))
    for target in action.target_locs:
        if action.target_locs[target] == -1:
            continue
        link = robot.link(FINGERTIP_LINK_NAMES[target])
        played_keys.append(target)
        bbox = link.geometry().getBBTight()

        # TODO need to find local target on fingertip
        local_p = vectorops.sub(get_fingertip(link),link.getWorldPosition(link.getMass().getCom()))
        world_p = action.target_locs[target]
        asc_world = [world_p[0], world_p[1], world_p[2] + height_offset]
        print("local point", local_p, "world point w/ height offset", asc_world)
        obj = ik.objective(link, local=local_p, world=asc_world)

        objectives.append(obj)

    finger_axis = vectorops.unit(vectorops.sub(robot.link(FINGERTIP_LINK_NAMES[2]).getTransform()[1], robot.link('ra_wrist_3_link').getTransform()[1]))
    obj = ik.fixed_rotation_objective(robot.link('rh_palm'), world_axis=[-1,0,0])
    objectives.append(obj)

    res = ik.solve_global(objectives, iters=200, numRestarts=200, feasibilityCheck=lambda : is_collision_free_chord(world, robot, played_keys, piano))
    

    action.delete_targets()

    if not res:
        print("play_chord ik unsuccessful")
        return None

    return robot.getConfig()

def disable_self_collisions(robot):
    for i in range(robot.numLinks()):
        for j in range(robot.numLinks()):
            if i != j:
                robot.enableSelfCollision(i, j, False)


def print_link_names(robot):
    for i in range(robot.numLinks()):
        print(robot.link(i).getName(), i)

def get_link_names(played_keys):
    link_names = []
    for finger_index in played_keys:
        link_names.append(FINGERTIP_LINK_NAMES[finger_index])
    return link_names


def is_collision_free_chord(world: WorldModel, robot: RobotModel, playing_keys: list, piano: Piano):
    #TODO: you might want to fix this to ignore collisions between finger pads and the object
    for i in range(world.numTerrains()):
        print(world.terrain(i).getName())
        for j in range(robot.numLinks()):
            if robot.link(j).geometry().collides(world.terrain(i).geometry()) and world.terrain(i).getName() != 'white_keys' and world.terrain(i).getName() != 'black_keys':
                print("Terrain collision found")
                print("Link name", robot.link(j).getName())
                print("Terrain name", world.terrain(i).getName())
                return False
    
    for i in range(robot.numLinks()):
        if robot.link(i).getName() in get_link_names(playing_keys):
            continue
        for j in range(world.numRigidObjects()):
            if robot.link(i).geometry().collides(world.rigidObject(i).geometry()):
                print("Rigid object collision found")
                return False

    print("No collision")
    return True

def load_model_world():
    world = WorldModel()
    dup_world = WorldModel()

    status = dup_world.readFile("world.xml")
    if not status:
        print("file not read")
        exit(-1)

    main_robot = dup_world.robot('Shadowhand')
    dup_robot = dup_world.robot('Shadowhand_dup')
    for i in range(main_robot.numLinks()):
        if main_robot.link(i).getName() in ARM_LINK_NAMES:
            main_robot.link(i).geometry().set(dup_robot.link(i).geometry().clone())
    
    world.add('Shadowhand', main_robot)
    return world

if __name__ == '__main__':
    robot_world = load_model_world()
    vis.add("world", robot_world)

    vis.run()
