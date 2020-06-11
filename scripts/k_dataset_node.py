#!/usr/bin/env python

import rospy
from load_data_utils import *
from publish_utils import *

import cv2
from cv_bridge import CvBridge
import numpy as np


from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Header


import os
DATA_PATH = '/home/anny/kittiRawData/2011_09_26/2011_09_26_drive_0005_sync/'



if __name__ == '__main__':
    
	rospy.init_node('kitti_node',anonymous=True)
	camera_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
	ego_car_pub = rospy.Publisher('kitti_ego_car', Marker, queue_size=10)
	car_module_pub = rospy.Publisher('kitti_car_module', Marker, queue_size=10)

	df = load_detection_result('/home/anny/kittiRawData/training/label_02/0000.txt')

	bridge = CvBridge()
	frame = 0	

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		boxes = np.array(df[df.frame==frame][['bbox_left' ,'bbox_top', 'bbox_right', 'bbox_bottom']])
		typs = np.array(df[df.frame==frame]['type'])

		img = load_image(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
		publish_camera(camera_pub, bridge, img, boxes, typs)

		point_cloud = load_pcl(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
		publish_lidar(pcl_pub, point_cloud)
		publish_ego_car(ego_car_pub)
		publish_car_module(car_module_pub)

		rospy.loginfo("pcl published")
		rate.sleep()
		frame += 1
		frame %= 154
