#!/usr/bin/python3
import os
import sys
sys.path.append('./neuralnetwork')

import time
import json 
import yaml
import argparse
import numpy as np
import cv2

import torch

import rospy
import ros_numpy
from cv_bridge import CvBridge

import struct
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from neuralnetwork.main import AnyNetModel, add_model_specific_args
from neuralnetwork.dataloader import preprocess


def add_args_from_launch(args):
    args.left_images = rospy.get_param('/anynet_ros_node/left_images')
    args.right_images = rospy.get_param('/anynet_ros_node/right_images')
    args.output_topic = rospy.get_param('/anynet_ros_node/output_topic')
    args.input_w = rospy.get_param('/anynet_ros_node/input_w')
    args.input_h = rospy.get_param('/anynet_ros_node/input_h')
    args.pretrained = rospy.get_param('/anynet_ros_node/checkpoint')
    args.with_spn = rospy.get_param('/anynet_ros_node/with_spn')
    args.disp2depth = rospy.get_param('/anynet_ros_node/disp2depth')
    args.focal_baseline = rospy.get_param('/anynet_ros_node/focal_baseline')
    return args


def config_args():
    parser = argparse.ArgumentParser()
    parser = add_model_specific_args(parser)
    return parser.parse_known_args()[0]


def preprocess_image(image):
    img = cv2.resize(image, (args.input_w, args.input_h))
    processed = preprocess.get_transform(augment=False)
    img = processed(img)
    return img


# See example http://wiki.ros.org/message_filters#Example_.28Python.29-1
def callback(imgL, imgR, verbose=False, quiet=False):
    if not quiet:
        rospy.loginfo(f'Got pair of images.\nLeft image info: {imgL.header}')

    if verbose:
        rospy.loginfo('input_msg height  : {}'.format(imgL.height))
        rospy.loginfo('input_msg width   : {}'.format(imgL.width))
        rospy.loginfo('input_msg step    : {}'.format(imgL.step))
        rospy.loginfo('input_msg encoding: {}'.format(imgL.encoding))

    imgL = br.imgmsg_to_cv2(imgL, desired_encoding='rgb8')
    imgR = br.imgmsg_to_cv2(imgR, desired_encoding='rgb8')

    # Preprocess
    imgL = preprocess_image(imgL)
    imgR = preprocess_image(imgR)

    # Predict disparity
    disparity = model.predict_disparity(imgL, imgR)
    if not args.disp2depth:
        disparity = disparity.cpu().numpy()[0, :, :]
    else:
        disparity = disparity[0, :, :]
        disparity[disparity != 0] = args.focal_baseline / disparity[disparity != 0]
        disparity = disparity.cpu().numpy()

    # Convert to msg and publish
    msg = br.cv2_to_imgmsg(disparity)
    pub_.publish(msg)

    if verbose:
        rospy.loginfo('-'*50)
        rospy.loginfo('output type       : {}'.format(type(disparity)))
        rospy.loginfo('output dtype      : {}'.format(disparity.dtype))
        rospy.loginfo('output shape      : {}'.format(disparity.shape))


if __name__ == "__main__":
    assert torch.cuda.is_available(), "Cuda seems not to work"
    
    # Init AnyNet ros node
    rospy.init_node('AnyNet_ros_node')

    # Config args
    args = config_args()
    args = add_args_from_launch(args)

    # Create model
    model = AnyNetModel(args)

    br = CvBridge()

    # Messages
    left_img_sub_ = message_filters.Subscriber(args.left_images, Image)
    right_img_sub_ = message_filters.Subscriber(args.right_images, Image)
    pub_ = rospy.Publisher(args.output_topic, Image, queue_size=1)

    # Syncronization
    ts = message_filters.ApproximateTimeSynchronizer([left_img_sub_, right_img_sub_], 1, 1)
    ts.registerCallback(callback)

    rospy.loginfo("[+] AnyNet ROS-node has started!")
    rospy.spin()