from klampt import WorldModel
from klampt import vis

def load_model_world():
    world = WorldModel()
    dup_world = WorldModel()

    status = dup_world.readFile("world.xml")
    if not status:
        print("file not read")
        exit(-1)

    link_names = ['ra_base_link', 'ra_shoulder_link', 'ra_upper_arm_link',\
                'ra_forearm_link', 'ra_wrist_1_link', 'ra_wrist_2_link', 'ra_wrist_3_link']

    main_robot = dup_world.robot('Shadowhand')
    dup_robot = dup_world.robot('Shadowhand_dup')
    for i in range(main_robot.numLinks()):
        if main_robot.link(i).getName() in link_names:
            main_robot.link(i).geometry().set(dup_robot.link(i).geometry().clone())
    
    world.add('Shadowhand', main_robot)
    return world

if __name__ == '__main__':
    robot_world = load_model_world()
    vis.add("world", robot_world)

    vis.run()
