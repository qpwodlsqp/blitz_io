import sys
import copy
import rospy
import actionlib
import moveit_commander

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from math import pi

class BlitzArmServer:

    def __init__(self):
        # initialize ur5 move_group
        moveit_commander.roscpp_initialize(sys.argv)
        self.ur5 = moveit_commander.MoveGroupCommander('ur5')

        # initialize robotiq gripper
        action_name = 'command_robotiq_action'
        self.gripper = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        self.gripper.wait_for_server()

        # Publisher to client
        self.pub = rospy.Publisher('ur_arm_finish', String, queue_size=10)

    def reverse_planning(self, plan):

        rev_plan = RobotTrajectory()
        rev_joint_traj = JointTrajectory()

        rev_joint_traj.header = copy.deepcopy(plan.joint_trajectory.header)
        rev_joint_traj.joint_names = copy.deepcopy(plan.joint_trajectory.joint_names)

        new_points = []
        points = plan.joint_trajectory.points # JointTrajectoryPoint[]
        l = len(points)

        '''
        How to reverse the cartesian plan?
        Reverse the order of trajectory point's joint values.
        Change the sign of velocity and acceleration.
        Change the accumulative time parameterization to follow
        the reverse order time gap.

        But you should put the current joint value to the first
        trajectory point. If not, the planner will return error due to
        joint deviation.
        '''
        for i in range(l):

            point = JointTrajectoryPoint()

            if i == 0:
                point.positions = self.ur5.get_current_joint_values()
                point.velocities = [0.0 for val in point.positions]
                point.accelerations = [0.0 for val in point.positions]
                point.time_from_start = rospy.Duration(0)
            else:
                point.positions = copy.deepcopy(points[l-1-i].positions)
                point.velocities = [-value for value in points[l-1-i].velocities]
                point.accelerations = [-value for value in points[l-1-i].accelerations]
                point.time_from_start = new_points[i-1].time_from_start + points[l-i].time_from_start - points[l-1-i].time_from_start

            new_points.append(point)

        rev_joint_traj.points = new_points
        rev_plan.joint_trajectory = rev_joint_traj
        return rev_plan

    def cartesian_planning(self, cmd):

        if (cmd[0] == 'PICK'):
            Robotiq.open(self.gripper, block=True)
            rospy.sleep(1)

        waypoint = []
        '''
        wpose is initialized twice to prevent inconsistent pose value
        which may happen when get_current_pose() is called first.
        '''
        wpose = self.ur5.get_current_pose().pose
        wpose = self.ur5.get_current_pose().pose
        waypoint.append(copy.deepcopy(wpose))

        # Move TCP to target position
        wpose.position.y += round(float(cmd[2]), 3)
        wpose.position.z += round(float(cmd[3]), 3) + 0.03
        wpose.position.x += round(float(cmd[1]), 3)
        waypoint.append(copy.deepcopy(wpose))

        wpose.position.z -= 0.03
        waypoint.append(copy.deepcopy(wpose))
        plan, fraction = self.ur5.compute_cartesian_path(waypoint, 0.01, 0.0)
        print('Fraction : {}'.format(fraction))

        self.ur5.execute(plan, wait=True)

        rospy.sleep(1)
        # Pick or Place or TCP Move Only
        if (cmd[0] == 'PICK'):
            Robotiq.close(self.gripper, block=True)
        elif (cmd[0] == 'PLACE'):
            Robotiq.open(self.gripper, block=True)
        else:
            return

        rospy.sleep(1)

        # Planning backward to the base pose

        rev_plan = self.reverse_planning(plan)
        self.ur5.execute(rev_plan, wait=True)
        self.ur5.stop()
        self.ur5.clear_pose_targets()
        print('cartesian planning done')

        return

    def base_pose(self):

        self.ur5.go([0.0, -0.2618, -2.618, -1.7596, -1.562, 2.356], wait=True)
        print('To Base')

        return

    def drive_pose(self):

        Robotiq.close(self.gripper, block=True)
        self.ur5.go([0.0, -0.2618, -2.618, 0.0, -1.562, 2.356], wait=True)
        print('To Drive')

        return

    def detection_pose(self):

        Robotiq.close(self.gripper, block=True)
        self.ur5.go([0.0, -0.2618, -2.618, -1.7596, -1.562, 1.570], wait=True)
        print('To Detection')

        return


    def handle(self, cmd):

        cmd = cmd.data.split()

        if (cmd[0] == 'DETECT'):
            self.detection_pose()
            rospy.sleep(0.5)
            return

        if (cmd[0] == 'DRIVE'):
            self.drive_pose()
            rospy.sleep(0.5)
            return

        if (cmd[0] == "BASE"):
            self.base_pose()
            rospy.sleep(0.5)
            return

        # self.base_pose()
        rospy.sleep(0.7)

        print("mode: {} , TCP move x: {} y: {} z: {}".format(cmd[0], cmd[1], cmd[2], cmd[3]))
        self.cartesian_planning(cmd)

        if cmd[0] != 'MOVE':
            rospy.sleep(1)
            self.drive_pose()
        rospy.sleep(2) # for communication consistency

        return


    def terminate(self):

        self.ur5.stop()
        print "\nBlitz Arm Server(UR5 Moveit & Robotiq Gripper) Shutdown"


def main():

    rospy.init_node('blitz_arm_server', anonymous=True)
    server = BlitzArmServer()
    rospy.on_shutdown(server.terminate)
    server.base_pose() # go to base pose
    server.drive_pose()

    while(True):

        try:
            print('command waiting ...')
            cmd = rospy.wait_for_message('ur_arm_control', String)
            server.handle(cmd)
            server.pub.publish('done')
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':

    main()
