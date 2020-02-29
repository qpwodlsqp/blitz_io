import sys
import rospy
from std_msgs.msg import String

'''
Execute server script before execute this client script!
'''
class BlitzPlanner:

    def __init__(self):

        self.arm_pub = rospy.Publisher('ur_arm_control', String, queue_size=1)
        print('Planner loaded')

    '''
    Send tcp position difference toward target pose to Moveit server in the Freight.
    x, y, z follow meter unit.
    client sleeps 1 sec for consistent communication with server
    '''
    def move_tcp_and_pick(self, x, y, z):

        cmd = 'PICK'+ " "+ str(x) + " " + str(y) + " "+ str(z)
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('tcp move and pick finished')
        return

    def move_tcp_and_place(self, x, y, z):

        cmd = 'PLACE'+ " "+ str(x) + " " + str(y) + " "+ str(z)
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('tcp move and place finished')
        return

    def move_tcp(self, x, y, z):

        cmd = 'MOVE'+ " "+ str(x) + " " + str(y) + " "+ str(z)
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('tcp move finished')
        return

    def detection_mode(self):

        cmd = 'DETECT'
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('moved to detection mode')
        return

    def drive_mode(self):

        cmd = 'DRIVE'
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('moved to drive mode')
        return

    def base_mode(self):

        cmd = 'BASE'
        rospy.sleep(1)
        self.arm_pub.publish(cmd)
        rospy.wait_for_message('ur_arm_finish', String)

        print('moved to base mode')
        return


########################################################################


def main():

    rospy.init_node('blitz_navigate_and_pick_pick_only', anonymous=True)
    planner = BlitzPlanner()
    planner.detection_mode()

    # planner.move_tcp_and_place(0.372, 0.103, 0.052)
    '''
    planner.move_tcp_and_pick(0.2, 0.1, 0.1)
    planner.move_tcp(0.2, 0.1, 0.1)
    planner.move_tcp(-0.2, -0.1, -0.1)
    planner.move_tcp_and_place(0.2, 0.1, 0.1)
    '''

if __name__ == '__main__':

    sys.exit(main())
