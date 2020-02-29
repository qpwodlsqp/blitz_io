import rospy
import sys
import actionlib

from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped

class BlitzNavigation:

    def __init__(self):

        self.nav_cli = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        print('Navigator loaded')

    def nav_goal(self, x, y, z, w):

        base_goal = MoveBaseGoal()
        target_pose = PoseStamped()

        header = Header()
        header.frame_id = 'map'

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        target_pose.header = header
        target_pose.pose = pose
        base_goal.target_pose = target_pose

        return base_goal

    def navigation(self, x, y, z, w):

        self.nav_cli.wait_for_server()
        goal = self.nav_goal(x, y, z, w)
        self.nav_cli.send_goal(goal)
        self.nav_cli.wait_for_result()
        nav_result = self.nav_cli.get_result()


###################################################################

import math
import tf

def q2a(z, w):

    return math.atan2(2.0*z*w, 1-2.0*z*z)

def main():

    rospy.init_node('blitz_navigate_and_pick_navigation_only', anonymous=True)    
    '''
    navigation start pose is initialized in config/amcl
    navigate to target pose
    '''
    start = (0.075, 1.915, -0.854, 0.520) # x, y, z, w
    navigator = BlitzNavigation()
    listener = tf.TransformListener()

    i = 0
    while i < 5:

        try:
            trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        x, y, _ = trans
        _, _, z, w = rot
        i += 1
        a = q2a(z, w)
        x += 0.1 * math.cos(a)
        y += 0.1 * math.sin(a)

        navigator.navigation(x, y, z, w)
        print('nav {} finished'.format(i))


if __name__ == "__main__":

    sys.exit(main())



