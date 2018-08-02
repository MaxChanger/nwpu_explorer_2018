#!/usr/bin/env python

# --------------------------------------------------------
# Faster R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""
# it's  two cameras' code

import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse

import rospy
from hector_worldmodel_msgs.msg import *
from ros_faster_rcnn.msg import *
from sensor_msgs.msg import CompressedImage,CameraInfo
from cv_bridge import CvBridge, CvBridgeError

RUNNING_left = False
IMAGE_left = CompressedImage()
Camera_left=CameraInfo()
RUNNING_right = False
IMAGE_right = CompressedImage()
Camera_right=CameraInfo()
	
def detect(image):
	"""Detect object classes in an image using pre-computed object proposals."""
	global NET
	
	# Detect all object classes and regress object bounds
	# rospy.loginfo("Starting detection")
	timer = Timer()
	timer.tic()
	scores, boxes = im_detect(NET, image)
	timer.toc()
	# rospy.loginfo('Detection took %f seconds for %d object proposals', timer.total_time, boxes.shape[0])
	return (scores, boxes)
			
def imageCallback_right(im):
	global IMAGE_right
	global RUNNING_right
	# rospy.loginfo('Image received')
	if (RUNNING_right):
		# rospy.logwarn('Detection already running, message omitted')
		rospy.logdebug('Detection already running, message omitted')
	else:
		#rospy.logerr('get a right image')
		RUNNING_right = True
		IMAGE_right = im

def imageCallback_left(im):
	global IMAGE_left
	global RUNNING_left
	# rospy.loginfo('Image received')
	if (RUNNING_left):
		# rospy.logwarn('Detection already running, message omitted')
		rospy.logdebug('Detection already running, message omitted')
	else:
		#rospy.logerr('get a left image')
		RUNNING_left = True
		IMAGE_left = im

def parse_args():
	"""Parse input arguments."""
	# Filter roslaunch arguments
	sys.argv = filter(lambda arg: not arg.startswith('__'), sys.argv)
	sys.argc = len(sys.argv)
	
	# Parse the other arguments
	parser = argparse.ArgumentParser(description='Faster R-CNN demo')
	parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
						default=0, type=int)
	parser.add_argument('--cpu', dest='cpu_mode',
						help='Use CPU mode (overrides --gpu)',
						action='store_true')
	parser.add_argument('--tresh', dest='treshold', help='The treshold for the detection', 
						default=0.5, type=float)
	parser.add_argument('--prototxt', dest='prototxt', help='The proto file', 
						default='../libraries/py-faster-rcnn/models/pascal_voc/ZF/faster_rcnn_alt_opt/faster_rcnn_test.pt')
	parser.add_argument('--model', dest='model', help='The model file', 
						default='../libraries/py-faster-rcnn/data/faster_rcnn_models/ZF_faster_rcnn_final.caffemodel')
						#default='../libraries/py-faster-rcnn/data/faster_rcnn_models/ZF_faster_rcnn_final_hdh.caffemodel')
	parser.add_argument('--classes', dest='classes', help='The file containing the classes', 
						default='classes.txt')
	parser.add_argument('--imageWindow_left', dest='imageWindow_left', help='left image window name', 
						default='Left Image window')
	parser.add_argument('--imageWindow_right', dest='imageWindow_right', help='right image window name', 
						default='Right Image window')
	parser.add_argument('--usb_cam_right', dest='usb_cam_right', help='ueb_cam_right', 
						default='/camera_right/')
	parser.add_argument('--usb_cam_left', dest='usb_cam_left', help='ueb_cam_left', 
						default='/camera_left/')
	parser.add_argument('--pub_name', dest='pub_name', help='the name of publish imagePerception', 
						default='image_percept')
	#or rcnn/res/perception,later we can change to the cool name
	args = parser.parse_args()
	
	return args
    
def parseClasses(classFile):
	with open(classFile) as f:
		content = f.readlines()
	return ['__background__'] + map(lambda x: x[:-1], content)

def generateDetections (scores, boxes, classes, threshold):
	# Visualize detections for each class
	global RUNNING_right
	NMS_THRESH = 0.3	
	res = []
	global bbox
	global score
	for cls_ind, cls in enumerate(classes[1:]):
		cls_ind += 1 # because we skipped background
		cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
		cls_scores = scores[:, cls_ind]
		dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
		keep = nms(dets, NMS_THRESH)
		dets = dets[keep, :]

		inds = np.where(dets[:, -1] >= threshold)[0]

		for i in inds:
			bbox = dets[i, :4]
			score = dets[i, -1]
			
			msg = Detection()
			#msg.header.frame_id = args.cameraSubscriber
			if (RUNNING_right):
				msg.header.frame_id ="/camera_right/image_raw/compressed"
			else:
				msg.header.frame_id ="/camera_left/image_raw/compressed"
			msg.x = bbox[0]
			msg.y = bbox[1]
			msg.width =  bbox[2] - bbox[0]
			msg.height = bbox[3] - bbox[1]
			msg.object_class = classes[cls_ind]
			msg.p = score
			res.append(msg)
	
	return res
	
def camaCallback_left(msg):
	global Camera_left
	Camera_left=msg

def camaCallback_right(msg):
	global Camera_right
	Camera_right=msg

def getResultImage (detections, image):
	font = cv2.FONT_HERSHEY_SIMPLEX
	textSize = cv2.getTextSize("test", font, 1, 2)
	delta = (textSize[1] * .3, textSize[1] * 2.4)
		
	for det in detections:
		cv2.rectangle(image, (det.x, det.y), (det.x + det.width, det.y + det.height), (0, 0, 255), 2)
		text = "{}: p={:.2f}".format(det.object_class, det.p)
		cv2.putText(image, text, (int(det.x + delta[0]), int(det.y + delta[1])), font, 0.8, (0, 0, 255), 2)
	return image
	
if __name__ == '__main__':
	cfg.TEST.HAS_RPN = True  # Use RPN for proposals
	args = parse_args()
	pub_perception= rospy.Publisher(args.pub_name, ImagePercept, queue_size = 10)
	#pub_perception=rospy.Pulisher('rcnn/res/perception',ImagePercept,queue_size=10)
	rospy.init_node('simpleDetect')
	# sub_image = rospy.Subscriber("rcnn/image_raw", Image, imageCallback)
	# sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, imageCallback)
	# sub_image = rospy.Subscriber(args.cameraSubscriber, Image, imageCallback)
	sub_image = rospy.Subscriber(args.usb_cam_left+"image_raw/compressed", CompressedImage, imageCallback_left)
	sub_caminfo= rospy.Subscriber(args.usb_cam_left+"camera_info", CameraInfo,camaCallback_left)
	sub_image = rospy.Subscriber(args.usb_cam_right+"image_raw/compressed", CompressedImage, imageCallback_right)
	sub_caminfo= rospy.Subscriber(args.usb_cam_right+"camera_info", CameraInfo,camaCallback_right)

	prototxt = os.path.join(os.path.dirname(__file__), args.prototxt)
	caffemodel = os.path.join(os.path.dirname(__file__), args.model)
	classes = parseClasses(os.path.join(os.path.dirname(__file__), args.classes))

	if not os.path.isfile(caffemodel):
		rospy.logerr('%s not found.\nDid you run ./data/script/fetch_faster_rcnn_models.sh?', caffemodel)

	if args.cpu_mode:
		caffe.set_mode_cpu()
	else:
		caffe.set_mode_gpu()
		caffe.set_device(args.gpu_id)
		cfg.GPU_ID = args.gpu_id
	NET = caffe.Net(prototxt, caffemodel, caffe.TEST)

	rospy.loginfo('Loaded network %s', caffemodel)
	rospy.loginfo('Running detection with these classes: %s', str(classes))
	rospy.loginfo('Warmup started')
	im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
	timer = Timer()
	timer.tic()
	for i in xrange(2):
		_, _= im_detect(NET, im)
	timer.toc()
	rospy.loginfo('Warmup done in %f seconds. Starting node', timer.total_time)
	
	rate = rospy.Rate(30)
	bridge = CvBridge()
	rospy.logerr(args.usb_cam_left+"image_raw/compressed")

	orderNum=0
	while not rospy.is_shutdown():
		if (RUNNING_left):
			rate.sleep()
			#cv_image = bridge.imgmsg_to_cv2(IMAGE)
			cv_image = bridge.compressed_imgmsg_to_cv2(IMAGE_left)
			cv_image=cv2.resize(cv_image,(640,480))
			(scores, boxes) = detect(cv_image)
			detections = generateDetections(scores, boxes, classes, args.treshold)
			
			for det in detections:
				rospy.loginfo(' [%s] detected, score: %.2f', det.object_class, det.p)
				rospy.loginfo(' Rectangle: (%d, %d), (%d, %d)', det.x, det.y, det.x + det.width, det.y + det.height)
				if(str(det.object_class)=="doll" and det.p >= 0.8):
		        		percp=ImagePercept()
		        		percp.header=IMAGE_left.header
		        		percp.camera_info=Camera_left
		        		percp.info.class_id = "victim"
		        		percp.info.class_support = 1.0;
		        		percp.info.object_id=str(det.header.seq)
		        		percp.info.object_support = 1.0;
		        		percp.info.name = det.object_class               
		        		percp.x = det.x + (det.width / 2)
		        		percp.y = det.y + (det.height / 2)
		        		percp.width = det.width
		        		percp.height = det.height
		        		pub_perception.publish(percp)
			cv2.imshow(args.imageWindow_left, getResultImage(detections, cv_image))
			cv2.waitKey(3)
			RUNNING_left = False
        #else:
			#rate.sleep()		
		elif(RUNNING_right):
			rate.sleep()
			#cv_image = bridge.imgmsg_to_cv2(IMAGE)
			cv_image = bridge.compressed_imgmsg_to_cv2(IMAGE_right)
			cv_image=cv2.resize(cv_image,(640,480))
			(scores, boxes) = detect(cv_image)
			detections = generateDetections(scores, boxes, classes, args.treshold)
			
			for det in detections:
				rospy.loginfo(' [%s] detected, score: %.2f', det.object_class, det.p)
				rospy.loginfo(' Rectangle: (%d, %d), (%d, %d)', det.x, det.y, det.x + det.width, det.y + det.height)
				if(str(det.object_class)=="doll"  and det.p >= 0.8):
		        		percp=ImagePercept()
		        		percp.header=IMAGE_right.header
		        		percp.camera_info=Camera_right
		        		percp.info.class_id = "victim"
		        		percp.info.class_support = 1.0
		        		percp.info.object_id=str(det.header.seq)
		        		percp.info.object_support = 1.0
		        		percp.info.name = det.object_class               
		        		percp.x = det.x + (det.width / 2)
		        		percp.y = det.y + (det.height / 2)
		        		percp.width = det.width
		        		percp.height = det.height
		        		pub_perception.publish(percp)
			cv2.imshow(args.imageWindow_right, getResultImage(detections, cv_image))
			cv2.waitKey(3)
			RUNNING_right = False
        else:
			rate.sleep()

