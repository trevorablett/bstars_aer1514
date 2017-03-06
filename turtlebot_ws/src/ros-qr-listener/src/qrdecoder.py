#!/usr/bin/env python

import PIL
import zbarlight
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class decoder:
	def __init__(self):
		self.image_pub = rospy.Publisher("qrcode",String,queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(rospy.get_param("qrdecoder/img_topic"),Image,self.callback)

	def callback(self,data):
		# convert sensor_msgs::Image to opencv
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# convert opencv image to pil image
		pil_im = PIL.Image.fromarray(cv_image)
		#pil_im.show()

		# scan codes
		codes = zbarlight.scan_codes('qrcode',pil_im)

		if codes != None:
			self.image_pub.publish(codes[0])


def main():
	d = decoder()
	rospy.init_node('QRDecoder', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
