from klampt import WorldModel, RobotModel
from klampt import vis
from klampt.model import ik

from piano import Piano
from music_functions import KeyAction

ARM_LINK_NAMES = ['ra_base_link', 'ra_shoulder_link', 'ra_upper_arm_link',\
            'ra_forearm_link', 'ra_wrist_1_link', 'ra_wrist_2_link', 'ra_wrist_3_link']

FINGERTIP_LINK_NAMES = ['rh_thdistal', 'rh_ffdistal', 'rh_mfdistal', 'rh_rfdistal', 'rh_lfdistal']


def get_finger_links(robot):
    links = {key: None for key in FINGERTIP_LINK_NAMES}
    for i in range(robot.numLinks()):
        if robot.link(i).getName() in FINGERTIP_LINK_NAMES:
            links[robot.link(i).getName()] = robot.link(i)
    
    return links

def play_chord(world: WorldModel, robot: RobotModel, piano: Piano, action: KeyAction, finger_links: list):
    action.convert_targets(piano)
    
    objectives = []
    for target in action.target_locs:
        if action.target_locs[target] == -1:
            continue
        bbox = finger_links[FINGERTIP_LINK_NAMES[target]].geometry().getBBTight()
        print(bbox)

        # TODO need to find local target on fingertip
        obj = ik.objective(finger_links[FINGERTIP_LINK_NAMES[target]], local=None, world=action.target_locs[target])


    action.delete_targets()


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
