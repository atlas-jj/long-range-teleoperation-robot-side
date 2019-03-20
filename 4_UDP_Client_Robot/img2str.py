#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('img2str')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import base64
import cStringIO
from PIL import Image as Image2


class Img2Str:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/img_str", String, queue_size = 1000)
		self.image_sub = rospy.Subscriber("/chris_cam/image_raw", Image, self.callback)
		print("subscribed")
		# while True:
		# 	self.image_pub.publish("asdfbase64 str")

	def callback(self, data):
		print("asfasd")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.img_to_str(cv_image)

	def img_to_str(self, img):
		print("asfasd")
		roi = cv2.resize(img, (640, 480))
		# # convert to gray scale
		roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
		# conver to pillow image
		pil_img = Image2.fromarray(roi_gray)
		buffer = cStringIO.StringIO()
		# conver to base64 encodes
		pil_img.save(buffer, format="JPEG")
		img_str = base64.b64encode(buffer.getvalue())
		# encoded_roi = base64.encodestring(np.ascontiguousarray(roi))
		cv2.imshow("base64str", roi_gray)
		cv2.waitKey(3)
		# self.image_pub.publish(encoded_roi)
		self.image_pub.publish(img_str)


def main(args):
	rospy.init_node('Img2Str', anonymous=True)
	ic = Img2Str()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
