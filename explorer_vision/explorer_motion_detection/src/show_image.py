#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import CompressedImage,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
bridge=CvBridge()
def ImageCallback(img):
	rospy.logerr("get image")
 	cv_image = bridge.compressed_imgmsg_to_cv2(img)
 	cv2.imshow("detect_image",cv_image)
	cv2.waitKey(3)



if __name__=="__main__":
	rospy.init_node("show_detect_image",anonymous=True)
	rospy.logerr("script start-----")
	rospy.Subscriber("/motion_detection/image_detected/compressed",CompressedImage,callback=ImageCallback)
	rospy.spin()
