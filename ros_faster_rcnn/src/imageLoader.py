#!/usr/bin/env python

import sys, cv2, rospy, os.path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


if __name__ == '__main__':
	image_pub = rospy.Publisher('/rcnn/image_raw', Image, queue_size = 1)
	rospy.init_node('imageLoader')
	bridge = CvBridge()

	for im_file in sys.argv[1:]:
		if (os.path.isfile(im_file)):
			cv_img = cv2.imread(im_file)
			img = bridge.cv2_to_imgmsg(cv_img)

			image_pub.publish(img)
			rospy.loginfo('Sending image % s', im_file)
		else:
			rospy.logerr("file '%s' does not exist", im_file)
	rospy.spin()

