#!/usr/bin/env python

import _init_paths
import os
import math
import random
import sys

import numpy as np
import tensorflow as tf
import cv2

import rospy
from hector_worldmodel_msgs.msg import *
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import argparse

slim = tf.contrib.slim

from nets import ssd_vgg_300, ssd_common, np_methods
from preprocessing import ssd_vgg_preprocessing
from notebooks import visualization

videoCam = cv2.VideoCapture(0)

gpu_options = tf.GPUOptions(allow_growth=True)
config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
isess = tf.InteractiveSession(config=config)

net_shape = (300, 300)
data_format = 'NHWC'
img_input =  tf.placeholder(tf.uint8, shape=(None, None, 3))

image_pre, labels_pre, bboxes_pre, bbox_img = ssd_vgg_preprocessing.preprocess_for_eval(
    img_input, None, None, net_shape, data_format, resize=ssd_vgg_preprocessing.Resize.WARP_RESIZE
)
image_4d = tf.expand_dims(image_pre, 0)

reuse = True if 'ssd_net' in locals() else None
ssd_net = ssd_vgg_300.SSDNet()
with slim.arg_scope(ssd_net.arg_scope(data_format=data_format)):
    predictions, localisations, _, _ = ssd_net.net(image_4d, is_training=False, reuse=reuse)

#ckpt_filename = '../checkpoints/VGG_VOC0712_SSD_300x300_ft_iter_120000.ckpt'
# ckpt_filename = '../checkpoints/ssd_300_vgg.ckpt'
isess.run(tf.global_variables_initializer())
saver = tf.train.Saver()
# saver.restore(isess, ckpt_filename)

ssd_anchors = ssd_net.anchors(net_shape)

def parse_args():

    sys.argv = filter(lambda arg: not arg.startswith('__'), sys.argv)
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description='SSD Train work')
    parser.add_argument('--model', dest='model', help='the path of model', 
                        default='', type=str)
    parser.add_argument('--cameraSubscriber', dest='camtopic', help='the topic of camera', 
                        default='/camera/image_raw/compressed', type=str)
    parser.add_argument('--numclass', dest='numclass', help='the number of classes', 
                        default=2, type=int)
    parser.add_argument('--pubtopic', dest='pubtopic', help='the topic which publish',
                        default='/camera/detected_frame', type=str)
    parser.add_argument('--caminfo', dest='caminfo', help='the matrix and info of camera',
                        default='/camera/camera_info', type=str)

    args = parser.parse_args()
    return args

def process_image(img, select_threshold=0.5, nms_threshold=.45, net_shape=(300, 300), numclass=21):
    rimg, rpredictions, rlocalisations, rbbox_img = isess.run([image_4d, predictions, localisations, bbox_img], feed_dict={img_input: img})
    rclasses, rscores, rbboxes = np_methods.ssd_bboxes_select(
        rpredictions, rlocalisations, ssd_anchors, 
        select_threshold=select_threshold, img_shape=net_shape, num_classes=numclass, decode=True
    )

    rbboxes = np_methods.bboxes_clip(rbbox_img, rbboxes)
    rclasses, rscores, rbboxes = np_methods.bboxes_sort(rclasses, rscores, rbboxes, top_k=400)
    rclasses, rscores, rbboxes = np_methods.bboxes_nms(rclasses, rscores, rbboxes, nms_threshold=nms_threshold)
    rbboxes = np_methods.bboxes_resize(rbbox_img, rbboxes)
    return rclasses, rscores, rbboxes

cam_image = CompressedImage()
cam_info = CameraInfo()

def imageCallback(image):
    global cam_image
    cam_image = image
    # cv_image = bridge.imgmsg_to_cv2(image)
    # cv2.imshow("test", cv_image)
def infoCallback(msg):
    global cam_info
    cam_info = msg

def detect(image, numclass):
    rclasses, rscores, rbboxes = process_image(img=image, numclass=numclass)
    detect_image = visualization.bboxes_draw_on_img(image, rclasses, rscores, rbboxes, visualization.colors_plasma)
    # cv2.imshow(detect_image)
    return detect_image

if __name__ == '__main__':
    args = parse_args()
    rospy.init_node('ssd_detect')
    rate = rospy.Rate(30)
    bridge = CvBridge()
    detect_pub = rospy.Publisher(args.pubtopic, Image, queue_size = 10)
    caminfo = rospy.Subscriber(args.caminfo, CameraInfo, infoCallback)
    # sub_image = rospy.Subscriber(args.camtopic, CompressedImage, imageCallback)
    sub_image = rospy.Subscriber(args.camtopic, CompressedImage, imageCallback)
    numclass = args.numclass
    # detect_pub = rospy.Publisher("/detect_frame", Image, queue_size=10)
    # sub_image = rospy.Subscriber("/camera/image_raw", Image, imageCallback)
    numclass = 2
    rospy.logwarn('ssd detect start')
    while not rospy.is_shutdown():
        rate.sleep()        
        if (cam_image.data):
            cv_image = bridge.compressed_imgmsg_to_cv2(cam_image, 'bgr8')
            # percept = ImagePercept()
            detect_pub.publish(bridge.cv2_to_imgmsg(detect(cv_image, numclass), 'bgr8'))
        else:
            rospy.logerr("no camera data has been put")

