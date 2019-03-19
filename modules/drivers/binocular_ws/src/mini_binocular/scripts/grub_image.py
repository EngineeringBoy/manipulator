#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import shlex
import subprocess

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MiniBinocularCamera:
    'mini binocular camera driver'
    def __init__(self, cam_id, calib_file):
        self.cam_id = cam_id
        cam = cv2.VideoCapture(self.cam_id)
        self.cam = cam
        self.cam_mode_dict = {
            'LEFT_EYE_MODE': 1,
            'RIGHT_EYE_MODE': 2,
            'RED_BLUE_MODE': 3,
            'BINOCULAR_MODE': 4,
        }
        self.cam_mode = self.cam_mode_dict['BINOCULAR_MODE']
        self.command_list = [
            "uvcdynctrl -d /dev/video{cam_id} -S 6:8  '(LE)0x50ff'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x00f6'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:8  '(LE)0x2500'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:8  '(LE)0x5ffe'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0003'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0002'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0012'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0004'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:8  '(LE)0x76c3'",
            "uvcdynctrl -d /dev/video{cam_id} -S 6:10 '(LE)0x0{cam_mode}00'",
        ]
        self.calib_file = calib_file
        self._camera=dict()
        self._stereo_rectify_map=self._get_rectify_map(calib_file)

        for init_num in range(3):
            for command in self.command_list:
                subprocess.Popen(shlex.split(command.format(cam_id=self.cam_id, cam_mode=self.cam_mode)))
            ret, frame = self.cam.read()
            cv2.waitKey(500)
        print 'calibration file path: ' + self.calib_file


    def grub(self):
        ret, frame = self.cam.read()

        if ret:
            expand_frame = cv2.resize(frame, None, fx=1, fy=0.5)
            return expand_frame
        else:
            return None

    def rectify_stereo(self,left_im,right_im):
        left_rect_im=self._rectify_image(left_im,
                                         self._stereo_rectify_map[0])
        right_rect_im = self._rectify_image(right_im,
                                            self._stereo_rectify_map[1])
        return left_rect_im, right_rect_im       

    def _rectify_image(self,image,rectify_map):
        rect_im=cv2.remap(image, rectify_map[0],
                          rectify_map[1],
                          cv2.INTER_LINEAR)
        return rect_im

    def _get_rectify_map(self, calib_file):
        self._read_calibration_file(calib_file)
        left_map=cv2.initUndistortRectifyMap(self._camera['K1'],
                                             self._camera['D1'],
                                             self._camera['R1'],
                                             self._camera['P1'],
                                             (640,240),
                                             cv2.CV_32F)
        right_map = cv2.initUndistortRectifyMap(self._camera['K2'],
                                                self._camera['D2'],
                                                self._camera['R2'],
                                                self._camera['P2'],
                                                (640,240),
                                                cv2.CV_32F)
        return [left_map,right_map]

    def _read_calibration_file(self, file_name):
        calib_file=cv2.FileStorage(file_name,flags=0)
        self._camera['K1']=np.array(calib_file.getNode('K1').mat())
        self._camera['K2'] = np.array(calib_file.getNode('K2').mat())
        self._camera['D1'] = np.array(calib_file.getNode('D1').mat())
        self._camera['D2'] = np.array(calib_file.getNode('D2').mat())

        self._camera['R1'] = np.array(calib_file.getNode('R1').mat())
        self._camera['R2'] = np.array(calib_file.getNode('R2').mat())
        self._camera['P1'] = np.array(calib_file.getNode('P1').mat())
        self._camera['P2'] = np.array(calib_file.getNode('P2').mat())
        self._camera['Q']= np.array(calib_file.getNode('Q').mat())
        self._camera['tilt']=0
        self._camera['height']=0
        self._camera['baseline']=np.array(calib_file.getNode('T').mat())[0]        

## grub images from mini binocular camera
def grub_image():
    image_pub = rospy.Publisher('/mini_binocular/image',Image, queue_size=1)
    rectify_pub = rospy.Publisher('/mini_binocular/rectify',Image, queue_size=1)
    bridge = CvBridge()

    rospy.init_node('grub_image', anonymous=True)
    calib_file = rospy.get_param("~calib_file")
    mini_binocular_cam = MiniBinocularCamera(1, calib_file)
    rate = rospy.Rate(30) # 30hz  

    while not rospy.is_shutdown():
        cv_image = mini_binocular_cam.grub()
        left_im = cv_image[:, 0:cv_image.shape[1] / 2, :]
        right_im = cv_image[:, cv_image.shape[1] / 2:, :]
        l_rect_im, r_rect_im = mini_binocular_cam.rectify_stereo(left_im, right_im)
        l_rect_im = l_rect_im[10:230, 10:310, :]
        r_rect_im = r_rect_im[10:230, 10:310, :]

        rectify_im = np.hstack((l_rect_im, r_rect_im))  

        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        rectify_pub.publish(bridge.cv2_to_imgmsg(rectify_im, 'bgr8'))
        rate.sleep()


if __name__ == '__main__':
    try:
        grub_image()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
