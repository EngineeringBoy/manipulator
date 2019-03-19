#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import cv2
import shlex
import subprocess

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MiniBinocularCamera:
    'mini binocular camera driver'
    def __init__(self, cam_id):
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
        for init_num in range(3):
            for command in self.command_list:
                subprocess.Popen(shlex.split(command.format(cam_id=self.cam_id, cam_mode=self.cam_mode)))
            ret, frame = self.cam.read()
            cv2.waitKey(500)


    def grub(self):
        ret, frame = self.cam.read()

        if ret:
            expand_frame = cv2.resize(frame, None, fx=1, fy=0.5)
            return expand_frame
        else:
            return None

## grub images from mini binocular camera
def grub_image():
    image_pub = rospy.Publisher('/mini_binocular/image',Image)
    bridge = CvBridge()

    rospy.init_node('grub_image', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    mini_binocular_cam = MiniBinocularCamera(1)

    while not rospy.is_shutdown():
        cv_image = mini_binocular_cam.grub()

        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        rate.sleep()


if __name__ == '__main__':
    try:
        grub_image()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
