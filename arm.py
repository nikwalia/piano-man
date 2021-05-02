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

    return vectorops.madd(pos, axis, finger_len)

def play_chord(world: WorldModel, robot: RobotModel, piano: Piano, action: KeyAction, playing_keys: list):
    action.convert_targets(piano)
    
    objectives = []
    for target in action.target_locs:
        if action.target_locs[target] == -1:
            # TODO maybe there's a way to set the non-playing fingers to be straight?
            # this would involve rotating those fingers and their parent links to be straight, then disabling those links
            # in ik.solve_global()
            continue
        link = playing_keys[FINGERTIP_LINK_NAMES[target]]
        bbox = link.geometry().getBBTight()

        # TODO need to find local target on fingertip
        obj = ik.objective(link, local=get_fingertip(link), world=action.target_locs[target])
        objectives.append(obj)
    
    res = ik.solve_global(objectives, iters=100, feasibilityCheck=lambda : is_collision_free_chord(world, robot, playing_keys, piano))

    action.delete_targets()

    if not res:
        return None

    return robot.getConfig()

def is_collision_free_chord(world: WorldModel, robot: RobotModel, playing_keys: list, piano: Piano):
    #TODO: you might want to fix this to ignore collisions between finger pads and the object
    if robot.selfCollides():
        return False
    for i in range(world.numTerrains()):
        for j in range(robot.numLinks()):
            if robot.link(j).geometry().collides(world.terrain(i).geometry()):
                return False

    # TODO need to add plank in piano model
    for i in range(robot.numLinks()):
        if robot.link(i).geometry().collides(piano.plank.geometry()):
            return False
    
    for i in range(robot.numLinks()):
        if robot.link(i).getName() in FINGERTIP_LINK_NAMES[playing_keys]:
            continue
        for j in range(world.numRigidObjects()):
            if robot.link(i).geometry().collides(world.rigidObject(i).geometry()):
                return False

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
