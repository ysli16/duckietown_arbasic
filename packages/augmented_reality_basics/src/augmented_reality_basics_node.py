#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 17:28:44 2020

@author: yueshan
"""
#import cv2

import augmented_reality_basics

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from duckietown.dtros import DTROS, NodeType

class AugmentedRealityBasics(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityBasics, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.bridge=CvBridge()
        #initialize Augmenter
        self.aug=augmented_reality_basics.Augmenter()
        # construct subscriber
        #self.sub_camerainfo = rospy.Subscriber("camera_node/camera_info", CameraInfo, self.cb_camerainfo)
        self.sub_img = rospy.Subscriber('camera_node/image/compressed', CompressedImage, self.cb_img)
        #constract publisher, topic is remapped to /robot name/node_name/map file basename/image/compressed in launch file
        self.pub_augmentedimg=rospy.Publisher('augmented_image', CompressedImage , queue_size=1)
        
        self.have_info=False
    def cb_img(self,msg):
        #rospy.loginfo("received an image")
        #convert compressedimage to cv2 image
        originalimg=self.bridge.compressed_imgmsg_to_cv2(msg)
        # draw segments using Augmenter
#        if self.have_info:
#            augmentedimg=self.aug.render_segments(originalimg,self.im_width,self.im_height)
#        else:
#            augmentedimg=self.aug.render_segments(originalimg)
        
        augmentedimg=self.aug.render_segments(originalimg)
        
        augmentedmsg=self.bridge.cv2_to_compressed_imgmsg(augmentedimg)
        #rospy.loginfo("augmented an image")
        self.pub_augmentedimg.publish(augmentedmsg)
        #rospy.loginfo("published an image")
#    def cb_camerainfo(self,msg):
#        self.have_info=True
#        self.im_width=msg.width
#        self.im_height=msg.height
        #rospy.loginfo("received camera info")
if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasics(node_name='augmented_reality_basics_node')

    # keep spinning
    rospy.spin()