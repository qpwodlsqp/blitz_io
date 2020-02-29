'''
Implementation of person following module in Pepper IO Robocup2018
to the Mobile Manipulator, a.k.a 'Blitz'.

Editor: Kim Jaein
Date: 2020/2/17

'''

import rospy
import math
import time
import cv2
import numpy as np
import copy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class BlitzFollower:

    def __init__(self, detector, time_limit=60.0, max_fail_count=10, safe_dist=1.0):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.time_limit = time_limit
        self.max_fail_count = max_fail_count
        self.safe_dist = safe_dist
        self.detector = detector # BlitzDetectiom
        self.max_miss_count = 5

        self.rgb_img = None
        self.depth_img = None

        return

    def find_target(self):

        # spin until both imgs are received
        while self.rgb_img is None or self.depth_img is None:
            pass

        r = self.detector.detection_image_input(self.rgb_img, target='person')
        rgb_img = copy.deepcopy(self.rgb_img)
        depth_img = copy.deepcopy(self.depth_img)

        if r is None:
            return None, None, None

        name, prob, box_info = self.detector.target_object(r, target='person')
        person_img = self.detector.crop(rgb_img, *box_info)

        self.rgb_img = None
        self.depth_img = None

        return (person_img, box_info, depth_img)

    def base_move_publish(self, lin, ang):

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_vel_pub.publish(twist)
        return

    def rgb_callback(self, img):

        try:
            rgb_img = self.detector.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError as e:
            print e
            return

        self.rgb_img = rgb_img
        return

    def depth_callback(self, img):

        try:
            depth_img = self.detector.bridge.imgmsg_to_cv2(img)
        except CvBridgeError as e:
            print e
            return

        self.depth_img = depth_img/1000.0
        return

    def follow_person(self):

        fail_count = 0
        prev  = None
        prev_ang = 0.0

        rospy.Subscriber('/xtion/rgb/image_rect_color', Image, self.rgb_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)
        start = time.time()

        # tmp failure exploration
        missed_count = 0

        while time.time() - start < self.time_limit:

            if fail_count > self.max_fail_count:

                print('*********Completely missed the target. Person Following Failed')
                return

            person_img, box_info, depth_img = self.find_target()

            # Detection failed
            if person_img is None or box_info is None or depth_img is None:

                if missed_count > self.max_miss_count:

                    print('**********detection missed')
                    missed_count = 0
                    fail_count += 1

                self.base_move_publish(0.0, prev_ang)
                continue

            # Whether detected target is one we were following
            if prev is not None:

                is_valid = self.is_valid_target(prev, person_img)

                if not is_valid:
                    print('**********not valid')
                    fail_count += 1
                    continue

            # prev = person_img
            x, y, w, h = box_info
            lin, ang = self.next_move(x, y, w, h, depth_img)
            self.base_move_publish(lin, ang)
            prev_ang = ang
            missed_count = 0
            fail_count = 0 # reset fail count
            print('***********base moved')

        return


    def next_move(self, x, y, w, h, depth_img):

        # default: (W, H) = (640, 480)
        W = self.detector.W
        H = self.detector.H

        # calculate rough relative target x y position to robot base
 
        person_center_range = depth_img[int(y-h/10):int(y+h/10), int(x-w/10):int(x+w/10)]
        base_x = np.mean(person_center_range)
        base_y = W/2 - x

        if base_x < self.safe_dist:
            lin = 0.0
        else:
            lin = 0.1

        if abs(base_y) < W / 20:
            ang = 0.0
        elif base_y > 0:
            ang = 0.2
        else:
            ang = -0.2

        return lin, ang

    def is_valid_target(self, prev, curr, thresh=0.1):

        h, w, _ = prev.shape
        curr = cv2.resize(curr, (w, h))

        prev = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
        curr = cv2.cvtColor(curr, cv2.COLOR_BGR2GRAY)

        prev_hist = cv2.calcHist(prev, [0], None, [50], [0, 256])
        curr_hist = cv2.calcHist(curr, [0], None, [50], [0, 256])
        correl = cv2.compareHist(prev_hist, curr_hist, cv2.cv.CV_COMP_CORREL)
        print(correl)

        if correl > thresh:
            return True
        else:
            return False

####################################

from detection import BlitzDetection
from parameter import get_param_dict

if __name__ == '__main__':

    rospy.init_node('blitz_follow_test', anonymous=True)
    param = get_param_dict()

    blitz_detector = BlitzDetection(param['cfg_path'],
                              param['weight_path'],
                              param['meta_path'],
                              param['W'],
                              param['H'],
                              param['camera_offset'],
                              param['tcp'])
    follower = BlitzFollower(detector=blitz_detector)

    print('Following start after 7 secs. It will move 60 secs')
    rospy.sleep(7)
    follower.follow_person()





