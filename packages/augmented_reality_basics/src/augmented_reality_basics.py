#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 17:30:37 2020

@author: yueshan
"""
import cv2 
import numpy as np
import yaml
import os
import rospy

from sensor_msgs.msg import CameraInfo
#from image_geometry import PinholeCameraModel
class Augmenter():
    def __init__(self):#load calibration files,mapfile
        self._points=rospy.get_param('~points')
        self._segments=rospy.get_param('~segments')    
        self.camerainfo=self.load_intrinsics()
#        self.pcm = PinholeCameraModel()
#        self.pcm.fromCameraInfo(self.camerainfo)
        self.points={}    
        self.homography=np.array(self.load_extrinsics()).reshape((3,3))
        self.Hinv=np.linalg.inv(self.homography)
        
        #rospy.loginfo("homography", type(self.homography))  
        
    def process_image(self,img):#undistort image using calibration files
        W = self.camerainfo.width
        H = self.camerainfo.height
        mapx = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapy = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(np.array(self.camerainfo.K).reshape((3,3)), 
                                                 np.array(self.camerainfo.D), 
                                                 np.array(self.camerainfo.R).reshape((3,3)),
                                                 None, 
                                                 (W, H),
                                                 cv2.CV_32FC1, mapx, mapy)
        cv_image_rectified = np.empty_like(img)
        res = cv2.remap(img, mapx, mapy, cv2.INTER_CUBIC,
                        cv_image_rectified)
        return res
        
        
    def ground2pixel(self,pt):#transform points in ground cordinates to pixels in image
        frame=pt[0]
        cord=pt[1]
        if frame=="image01":
            pos_u=int(cord[1]*self.camerainfo.width)
            pos_v=int(cord[0]*self.camerainfo.height)
            return [pos_u,pos_v]
        elif frame=="axle":
            ground_point = np.array([cord[0], cord[1], 1.0])
            #rospy.loginfo("got ground_point")
            image_point = np.dot(self.Hinv, ground_point)
            image_point = image_point / image_point[2]
            image_point=image_point.astype(int)
            #rospy.loginfo("got image_point")
            return image_point[0:2]
        else:
            pass
    def process_points(self)  :
        #convert all points to pixels
        for key in self._points:
            self.points[key]=self.ground2pixel(self._points[key])
            
    def render_segments(self,img):#draw segments according to map file onto image          
        self.process_points()
        img=self.process_image(img)
        for segment in self._segments:
            point1=self.points[segment['points'][0]]
            point2=self.points[segment['points'][1]]
            point_x=[point1[0],point2[0]]
            point_y=[point1[1],point2[1]]
            color=segment['color']
            img=self.draw_segment(img,point_x,point_y,color)
        return img
    
    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
        return image
    
    def load_intrinsics(self):
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        
        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        try:
            with open(cali_file,'r') as stream:
                rospy.loginfo("opened intrinsics calibration file")
                calib_data = yaml.load(stream)
        except yaml.YAMLError:
            msg = 'Error in parsing calibration file %s ... aborting' % cali_file
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)
            
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
        
    def load_extrinsics(self):
        """
        Loads the homography matrix from the extrinsic calibration file.
        Returns:
            :obj:`numpy array`: the loaded homography matrix
        """
        # load extrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        try:
            with open(cali_file,'r') as stream:
                rospy.loginfo("opened extrinsics calibration file")
                calib_data = yaml.load(stream)
        except yaml.YAMLError:
            msg = 'Error in parsing calibration file %s ... aborting' % cali_file
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)
          
        return calib_data['homography']
