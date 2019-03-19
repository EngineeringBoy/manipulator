#!/usr/bin/env python
import os
import sys
import numpy as np
import cv2
import rospy
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

c_width = 11
c_height = 8
save_calib_img_dir = 'data/'
save_img_cnt = 1
class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/mini_binocular/image', Image, self.callback)
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)
		chess_im = draw_chessboard(cv_image)
		cv2.imshow("Image window", chess_im)

		if cv2.waitKey(3) & 0xff == ord('s'):
			global save_img_cnt
			filename1 = "%sleft%d.%s" % (save_calib_img_dir, save_img_cnt, 'png')
			filename2 = "%sright%d.%s" % (save_calib_img_dir, save_img_cnt, 'png')
			cv2.imwrite(filename1, cv_image[:, 0:cv_image.shape[1]/2, :])
			cv2.imwrite(filename2, cv_image[:, cv_image.shape[1]/2:, :])
			print "save image: " + filename1 + ', ' + filename2
			save_img_cnt += 1

def draw_chessboard(image):
		l_im = copy.deepcopy(image[:, 0:image.shape[1] / 2, :])
		r_im = copy.deepcopy(image[:, image.shape[1] / 2:, :])
		res, corners = cv2.findChessboardCorners(l_im, (c_width, c_height))
		if res:
			cv2.drawChessboardCorners(l_im, (c_width, c_height), corners, True)
		res, corners = cv2.findChessboardCorners(r_im, (c_width, c_height))
		if res:
			cv2.drawChessboardCorners(r_im, (c_width, c_height), corners, True)
		return np.concatenate((l_im,r_im),1)

def main(args):
	global c_width, c_height, save_calib_img_dir
	ic = image_converter()
	rospy.init_node('save_calibration_image', anonymous=True)
	c_width = rospy.get_param('~calib_board_width')
	c_height = rospy.get_param('~calib_board_height')
	save_calib_img_dir = rospy.get_param('~save_calib_img_dir')
	if not os.path.exists(save_calib_img_dir):
		os.makedirs(save_calib_img_dir)
	print 'calibration images save directory is:' + save_calib_img_dir
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
 
if __name__ == '__main__':
	main(sys.argv)
