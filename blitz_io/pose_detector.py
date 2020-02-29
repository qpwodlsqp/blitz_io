import cv2
import numpy as np
import pyopenpose as op
import copy

class PoseDetector:

    def __init__(self, thresh=0.1, visualize=False):

        params = dict()
        params['model_pose'] = 'MPI'
        params['model_folder'] = "/home/user/kji/openpose_model/"

        self.openpose = op.WrapperPython()
        self.openpose.configure(params)
        self.openpose.start()
        self.thresh=thresh
        self.visualize = visualize

        # MPI model
        self.body_dict = {
                            "Head" : 0,
                            "Neck" : 1,
                            "RShoulder" : 2,
                            "RElbow" : 3,
                            "RWrist" : 4,
                            "LShoulder" : 5,
                            "LElbow" : 6,
                            "LWrist" : 7,
                            "RHip" : 8,
                            "RKnee" : 9,
                            "RAnkle" : 10,
                            "LHip" : 11,
                            "LKnee" : 12,
                            "LAnkle" : 13,
                            "Chest" : 14,
                            "Background" : 15
                         }

        return

    # Detect whether arm pose is curved or raised
    def arm_pose(self, shoulder, elbow, wrist):

        RAISED = False
        CURVED = False

        if shoulder[1] < elbow[1] and wrist[1] < elbow[1]:
            CURVED = True
        elif shoulder[1] > elbow[1] and elbow[1] > wrist[1]:
            RAISED = True

        return (CURVED, RAISED)

    def isWaving(self, body_points_list):

        FAIL_LIMIT = (len(body_points_list) * 0.4)
        FAIL_COUNT = 0

        R_CURVED = False
        R_RAISED = False
        L_CURVED = False
        L_RAISED = False

        for body_points in body_points_list:
            rshoulder = body_points[self.body_dict['RShoulder']]
            relbow = body_points[self.body_dict['RElbow']]
            rwrist = body_points[self.body_dict['RWrist']]

            lshoulder = body_points[self.body_dict['LShoulder']]
            lelbow = body_points[self.body_dict['LElbow']]
            lwrist = body_points[self.body_dict['LWrist']]

            if rshoulder[2] > self.thresh and relbow[2] > self.thresh and rwrist[2] > self.thresh :

                moved_here = True
                curved, raised = self.arm_pose(rshoulder, relbow, rwrist)
                if (curved == False and raised == False):
                    # Right arm seems to not related with waving. Check left arm then.
                    moved_here = False
                    pass
                elif (R_CURVED != R_RAISED) and ((R_CURVED != curved) and (R_RAISED != raised)):
                    # This logic computes whether arm pose is changed between frames.
                    # This provides to check arm up and down pose
                    FAIL_COUNT += 1
                    print('Arm pose is changing. Fail Count Increment')
                else:
                    # wave success
                    print('Right Arm Waved')

                R_CURVED = curved
                R_RAISED = raised

                if moved_here:
                    continue

            if lshoulder[2] > self.thresh and lelbow[2] > self.thresh and lwrist[2] > self.thresh :

                moved_here = True
                curved, raised = self.arm_pose(lshoulder, lelbow, lwrist)
                if (curved == False and raised == False):
                    # Left arm seems to not related with waving, too. Fail Cound increments then.
                    moved_here = False
                elif (L_CURVED != L_RAISED) and ((L_CURVED != curved) and (L_RAISED != raised)):
                    FAIL_COUNT += 1
                    print('Arm pose is changing. Fail Count Increment')
                else:
                    print('Left Arm Waved')

                L_CURVED = curved
                L_RAISED = raised

                if moved_here:
                    continue

            print("Can't detect both arm's proper movement")
            FAIL_COUNT += 1


        if FAIL_COUNT > FAIL_LIMIT:
            return False
        else:
            return True


    def predict(self, N):

        # input cv_img_list contains person cropped images

        body_points_list = []

        for i in range(N):

            person_img = cv2.imread('hand_waving_frames/person_frame_{}.jpg'.format(i))
            datum = op.Datum()
            datum.cvInputData = person_img
            self.openpose.emplaceAndPop([datum])

            print("Body keypoints: \n" + str(datum.poseKeypoints))

            if datum.poseKeypoints.shape != (1, 15, 3):
                continue

            body_points = datum.poseKeypoints[0] # (1, #BODY_PARTS, 3) (x, y, prob)
            body_points_list.append(body_points)

        
            if self.visualize:
                cv2.imwrite('hand_waving_results/pose_visualize_{}.jpg'.format(i), datum.cvOutputData)

        return self.isWaving(body_points_list)

    '''
    def test(self, cv_img):

        datum = op.Datum()
        datum.cvInputData = cv_img
        self.openpose.emplaceAndPop([datum])

        keypoint_list = datum.poseKeypoints
        print("Body keypoints: \n" + str(keypoint_list))
        print(keypoint_list.shape)
        cv2.imshow('openpose test', datum.cvOutputData)
        cv2.waitKey(0)
    '''
############################################################

if __name__ == '__main__':

    pose_detector = PoseDetector()
    pose_detector.test()
