import sys
import rospy
import math

from navigation import BlitzNavigation
from detection import BlitzDetection
from planning import BlitzPlanner
from parameter import get_param_dict
from reachability import BlitzReachability

def main():

    rospy.init_node('blitz_navigate_and_pick', anonymous=True)

    param = get_param_dict()
    planner = BlitzPlanner()
    navigator = BlitzNavigation()
    reach_map = BlitzReachability()
    detector = BlitzDetection(param['cfg_path'],
                              param['weight_path'],
                              param['meta_path'],
                              param['W'],
                              param['H'],
                              param['camera_offset'],
                              param['tcp'])
    
    # navigate to target location
    # target_loc = raw_input('navigate to (dining, kitchen, corridor) :')
    print("Navigate to corridor and find the teddy bear.")
    target_loc = "kitchen"
    navigator.navigation(*param[target_loc])
    
    # object detection & calculate tcp difference

    differ = (0.0, 0.0, 0.0)
    r = []
    target_nav = param[target_loc]
    a_ = 0.0
    while (True):
        planner.detection_mode()
        rospy.sleep(1.5)
        r = detector.detection()

        # TODO : exploration when can't find target object(teddy bear)
        if r is not None: break

        if a_ >= round(2 * math.pi):
            print("There aren't any target in this room")
            planner.drive_mode()
            return

        planner.drive_mode()
        rospy.sleep(0.3)

        x, y, z, w = target_nav
        a = reach_map.quaternion_to_yaw(z, w)
        a_ += math.pi / 2
        z, w = reach_map.yaw_to_quaternion(a+a_)
        target_nav = (x, y, z, w)
        navigator.navigation(*target_nav)

    target, _, box_info = detector.target_object(r)
    cloud = detector.detected_cloud(target, box_info)
    object_xyz = detector.target_position(cloud)

    nav_pose,object_position = reach_map.adjust_base(target_nav,object_xyz)

    print("------------adjust navigation pose----------------")
    print(nav_pose)
    print("------------adjust object position----------------")
    print(object_position)
    planner.drive_mode()
    navigator.navigation(*nav_pose)
    rospy.sleep(1)

    '''
    # calculate once
    differ = detector.tcp_calc(*object_position)
    '''

    # detect again
    planner.detection_mode()
    rospy.sleep(1)
    r = detector.detection()
    target, _, box_info = detector.target_object(r)
    cloud = detector.detected_cloud(target, box_info)
    object_xyz = detector.target_position(cloud)
    differ = detector.tcp_calc(*object_xyz)

    planner.base_mode()
    planner.move_tcp_and_pick(*differ)

    rospy.sleep(1)
    # place object
    planner.base_mode()
    planner.move_tcp_and_place(*differ)


if __name__ == '__main__':

    sys.exit(main())



