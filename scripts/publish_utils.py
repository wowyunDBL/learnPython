#!/usr/bin/env python
import rospy
import rospkg
import cv2
import tf
import numpy as np
from std_msgs.msg import Header,String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2    #convert .bin file to ros msg


FRAME_ID = 'map'
COLOR_BOX_DICT = {'Car':(255,255,0), 'Pedestrian':(0,226,255), 'Cyclist':(141,40,255)}

def publish_camera(camera_pub, bridge, image, boxes, typs):
	for typ,box in zip(typs,boxes):
	    	top_left = int(box[0]),int(box[1])
	    	bottom_right = int(box[2]),int(box[3])
	    	cv2.rectangle(image, top_left, bottom_right,COLOR_BOX_DICT[typ],2)
	camera_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


def publish_lidar(pcl_pub, point_cloud):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = FRAME_ID
	pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))   #all row and the first 3 column

def publish_ego_car(ego_car_pub):
	marker = Marker()
	marker.header.frame_id = FRAME_ID
	marker.header.stamp = rospy.Time.now()
	marker.type = Marker.LINE_STRIP
	marker.id = 0
	marker.action = Marker.ADD

	marker.scale.x = 0.1
	marker.color.b = 1.0
	marker.color.a = 1.0
	
	marker.points = []
	marker.points.append(Point(10,10,0))
	marker.points.append(Point(0,0,0))
	marker.points.append(Point(10,-10,0))
	
	ego_car_pub.publish(marker)

def publish_car_module(car_module_pub):
	mesh_marker = Marker()
	mesh_marker.header.frame_id = FRAME_ID
	mesh_marker.header.stamp = rospy.Time.now()
	mesh_marker.type = Marker.MESH_RESOURCE
	rospack = rospkg.RosPack()
	string = rospack.get_path('pylearning')
	mesh_marker.mesh_resource = "file:///home/anny/kitti_tutor/catkin_ws/src/pylearning/Car-Model/BMW X5 4.dae"
	mesh_marker.id = -1
	mesh_marker.action = Marker.ADD

	q = tf.transformations.quaternion_from_euler(np.pi/2,0,np.pi)
	mesh_marker.pose.orientation.x = q[0]
	mesh_marker.pose.orientation.y = q[1]
	mesh_marker.pose.orientation.z = q[2]
	mesh_marker.pose.orientation.w = q[3]

	mesh_marker.scale.x = 1
	mesh_marker.scale.y = 1
	mesh_marker.scale.z = 1

	mesh_marker.color.g = 1.0
	mesh_marker.color.a = 1.0
	
	mesh_marker.pose.position.x = 0
	mesh_marker.pose.position.y = 0
	mesh_marker.pose.position.z = -1.75
	
	car_module_pub.publish(mesh_marker)
