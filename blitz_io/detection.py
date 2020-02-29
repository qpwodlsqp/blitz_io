import rospy
import ros_numpy
import cv2
import darknet
import math
import numpy as np

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from cv_bridge import CvBridge, CvBridgeError
from pose_detector import PoseDetector
import sensor_msgs.point_cloud2 as pc2

class BlitzDetection:

    def __init__(self, cfg, weight, meta, W, H, camera_offset, tcp):

        self.net = darknet.load_net(cfg, weight, 0)
        self.meta = darknet.load_meta(meta)
        self.bridge = CvBridge()
        # receiving image's resolution
        self.W = W
        self.H = H
        # camera's position referencing base_link
        self.CAMERA_OFFSET = camera_offset
        # tcp
        self.TCP = tcp
        # Openpose Pose Detector
        self.pose_detector = PoseDetector(thresh=0.5, visualize=True)

        print('Detector loaded')

    def get_image(self, camera='/camera/color/image_raw'):

        img = rospy.wait_for_message(camera, Image)
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError as e:
            print e

        print "Image Received"
        return cv2_img

    '''
    def get_depth_image(self):

        depth_img = rospy.wait_for_message('/xtion/depth/image_raw', Image)
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_img) # depth in mm unit
        except CvBridgeError as e:
            print e

        return depth_img/1000.0 # change to m unit
    '''

    def detection_all(self, thresh=0.7):

        cv_img = self.get_image()
        r = darknet.darknet(self.net, self.meta, cv_img)

        if len(r) == 0:
            print 'Could not detect anything'
            return None

        detection_list = []
        for item in r:
            name, prob, box_info = item
            if prob >= thresh:
                print '{} detected!'.format(name)
                detection_list.append(item)

        return detection_list

    def detection(self, target='teddy bear'):

        cv2_img = self.get_image()
        r = darknet.detect(self.net, self.meta, cv2_img)

        if len(r) == 0:
            print "Could not detect anything"
            return None

        for item in r:
            if target in item:
                print "Found teddy bear in the image"
                return r
            else:
                pass

        print "No teddy bear in this image"
        return None

    def detection_image_input(self, cv_img, target='teddy bear'):

        r = darknet.detect(self.net, self.meta, cv_img)

        if len(r) == 0:
            print "Could not detect anything"
            return None

        for item in r:
            if target in item:
                print "Found {} in the image".format(target)
                return r
            else:
                pass

        print "No {} in this image".format(target)
        return None

    def target_object(self, r, target='teddy bear'):

        for item in r:
            name, prob, box_info = item
            print(name)

        return [item for item in r if target in item][0]

    def detected_cloud(self, target, box_info):

        cloud = ros_numpy.numpify(rospy.wait_for_message('/camera/depth_registered/points', PointCloud2))

        target_cloud = []
        for i in range(self.H):
            for j in range(self.W):

                point = cloud[i, j]

                if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
                    target_cloud.append((0, 0, 0))
                    continue

                (x, y, w, h) = box_info
                if j >= x - w/2 and j <= x + w/2 and i >= y - h/2 and i <= y + h/2:
                    target_cloud.append((point[0], point[1], point[2]))
                else:
                    target_cloud.append((0, 0, 0))

        # visualize target's point cloud

        '''
        fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                 ]
        header = Header()
        header.frame_id = 'camera_depth_optical_frame'
        pub = rospy.Publisher('arm_move_point', PointCloud2, queue_size=2)
        pub.publish(pc2.create_cloud(header, fields, target_cloud))
        '''
        return target_cloud

    def target_position(self, cloud):

        point_list = []

        for point in cloud:

            if point == (0, 0, 0): continue
            else:
                x = self.CAMERA_OFFSET[0] + point[2]
                y = self.CAMERA_OFFSET[1] - point[0]
                z = self.CAMERA_OFFSET[2] - point[1]
                point_list.append((x, y, z))

        sorted_by_depth = sorted(point_list, key=lambda point: point[0])
        object_list = sorted_by_depth[:len(sorted_by_depth)/2]

        x = sum([item[0] for item in object_list])/len(object_list)
        y = sum([item[1] for item in object_list])/len(object_list)
        z = sum([item[2] for item in object_list])/len(object_list)

        return (x, y, z)

    def tcp_calc(self, x, y, z):

        differ = (x-self.TCP[0], y-self.TCP[1], z-self.TCP[2])

        return differ

    def crop(self, cv_img, x, y, w, h):

        cropped = cv_img[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)]
        return cropped

    def hand_waving_callback(self, img):

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError as e:
            print e
            return

        self.hand_wave_img = rgb_img
        return

    def is_hand_waving(self):

        N = 10
        i = 0
        '''
        rate = rospy.Rate(5)
        rospy.Subscriber('/xtion/rgb/image_rect_color', Image, self.hand_waving_callback)        
        '''
        while i < N:

            '''
            while self.hand_wave_img is None:
                pass
            '''
            hand_wave_img = self.get_image(camera='/xtion/rgb/image_rect_color')
            r = self.detection_image_input(hand_wave_img, 'person')
            
            if r is None:
                continue

            name, prob, (x, y, w, h) = self.target_object(r, target='person')
            person_img = self.crop(hand_wave_img, x, y, w, h)
            # Directly putting briged img to Openpose causes fcuked up results
            cv2.imwrite('hand_waving_frames/person_frame_{}.jpg'.format(i), person_img)
            i += 1

        is_waving = self.pose_detector.predict(N)

        if is_waving:
            print('A person is waving hand')
        else:
            print('A person is not waving hand')

        return
######################################################################

import time

def main():

    CFG_PATH = "/home/user/kji/darknet/cfg/yolov3.cfg"
    WEIGHT_PATH = "/home/user/kji/darknet/backup/yolov3.weights"
    META_PATH = "/home/user/kji/darknet/cfg/coco.data"
    W = 640
    H = 480
    camera_offset = (0.5, -0.1, 0.98)
    tcp = (0.72, -0.108, 0.905)

    detector = BlitzDetection(CFG_PATH, WEIGHT_PATH, META_PATH, W, H, camera_offset, tcp)

    rospy.init_node('blitz_navigate_and_pick_detection', anonymous=True)

    # Target Detection
    '''
    r = detector.detection()
    target, prob, box_info = detector.target_object(r)
    cloud = detector.detected_cloud(target, box_info)
    x, y, z = detector.target_position(cloud)

    print(x)
    print(y)
    print(z)

    x, y, z = detector.tcp_calc(x, y, z)

    print(x)
    print(y)
    print(z)
    '''

    # Hand Waving
    print('Get ready to wave your hands after 7 secs')
    tic = time.time()
    rospy.sleep(7)
    detector.is_hand_waving()
    toc = time.time()
    print('Time Cost: {}'.format(toc-tic))

    '''
    depth = detector.get_depth_image()

    print(depth.shape)
    print(depth[240, 320])
    '''

if __name__ == '__main__':

    main()

