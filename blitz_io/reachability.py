import rospy
import math
from map_creator.msg import WorkSpace, WsSphere

class BlitzReachability:

    def __init__(self):

        self.spheres = []
        voxels = rospy.wait_for_message("/reachability_map", WorkSpace).WsSpheres
        min_rad = float("inf")
        max_rad = 0.0

        for voxel in voxels:

            if voxel.ri < 90 or voxel.point.x < 0:
                continue

            self.spheres.append(voxel)
            center = voxel.point # referencing arm_base_link x+0.05 z+0.95 to base_link
            rad = math.sqrt((center.x**2) + (center.y**2) + (center.z**2))

            if rad > max_rad:
                max_rad = rad

            if rad < min_rad:
                min_rad = rad

        self.min_rad = min_rad
        self.max_rad = max_rad
        print('Reachability Map Loaded')

        return

    def get_reachability_map(self):

        return self.spheres

    def reachability_ground_range(self):

        return self.min_rad, self.max_rad

    def yaw_to_quaternion(self, a):

        # the function is abbreviated because roll & pitch are always zero.

        return (round(math.sin(a * 0.5), 4), round(math.cos(a * 0.5), 4)) # (z, w)

    def quaternion_to_yaw(self, z, w):

        siny = 2.0 * z * w
        cosy = 1 - (2 * z**2)
        return math.atan2(siny, cosy)

    def adjust_base(self, base_pose, target_position):

        x, y, z, w = base_pose
        target = (target_position[0], target_position[1], target_position[2]-0.95)

        rad = math.sqrt(target[0]**2 + target[1]**2 + target[2]**2)

        # get into the high reachability zone
        if rad >= self.min_rad and rad <= self.max_rad:
            d = 0.0
        elif rad > self.max_rad:
            d = rad - self.max_rad
        elif rad < self.min_rad:
            d = rad - self.min_rad

        # aligin target to arm_base_link
        a = self.quaternion_to_yaw(z, w)
        a_ = math.atan(target[1]/target[0])
        x_ = d*math.cos(a+a_)
        y_ = d*math.sin(a+a_)
        new_z, new_w = self.yaw_to_quaternion(a + a_)

        '''
        t_x = d*math.cos(a_)
        t_y = d*math.sin(a_)

        target_position = (target_position[0]-t_x, target_position[1]-t_y, target_position[2])
        '''

        # additional adjustment to aligin target's y to tcp
        # -0.108 is tcp's y value.
        delta_x = -0.108 * math.sin(a+a_)
        delta_y = 0.108 * math.cos(a+a_)

        target_position = (rad - d, -0.108, target_position[2])
        base_pose = (x + x_ + delta_x, y + y_ + delta_y, new_z, new_w)

        return base_pose, target_position



##########################################################################################


def main():

    rospy.init_node('blitz_reachability_map')
    reach = BlitzReachability()
    min_rad, max_rad = reach.reachability_ground_range()
    print(min_rad)
    print(max_rad)

if __name__ == '__main__':

    main()
