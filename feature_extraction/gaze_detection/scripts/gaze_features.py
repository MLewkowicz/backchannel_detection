#!/usr/bin/env python

import numpy as np
from rospywrapper import TopicSource, BagSource, Synchronizer

import rospy
from rospy_msg_converter import convert_ros_message_to_dictionary
from openface_ros.msg import face_info, pose
from gaze_objects import Jibo, Camera, Face
from config import get_configuration
import message_filters

def main():
	(bag, gaze_sources, features_sink, start_time, window_duration) = get_configuration()
	camera_source_1, camera_source_2, camera_source_3 = nod_src[0], nod_src[1], nod_src[2]
	jibo = Jibo([0,0,0])
	camera1 = Camera('camera1', [90, -254, 36], -20.0, 54.462)
	camera2 = Camera('camera2', [230, 4, 36], -20.0, 0)
	camera3 = Camera('camera3', [85, 247, 36], -20.0, 326.301)
	with camera_source_1, camera_source_2, camera_source_3, features_sink:
		sync = Synchronizer([camera1topic, camera2topic, camera3topic])
		for (face_info_1, face_info_2, face_info_3) in sync:
			poses = []
			for x in (face_info_1, face_info_2, face_info_3):
				msg = convert_ros_message_to_dictionary(x[0][0])
				if msg['detected'] == 1:
			 		poses.append(msg['head_pose'])
			if len(poses) == 2:
				pose1, pose2 = poses[0], poses[1]
				p1 = Face(camera1,
						[pose1['x'], pose1['y'], pose1['z']],
						pose1['rot_x'], pose1['rot_y'], pose1['rot_z'])
				p1.convertFaceFromLocalToWorldSpace()
				p2 = Face(camera2,
							[pose2['x'], pose2['y'], pose2['z']],
							pose2['rot_x'], pose2['rot_y'], pose2['rot_z'])
				rospy.loginfo(p1.position)
				rospy.loginfo(p2.position)
			if len(poses) == 3:
				pose1, pose2, pose3 = poses[0], poses[1], poses[2]
				p1 = Face(camera1,
						[pose1['x'], pose1['y'], pose1['z']],
						pose1['rot_x'], pose1['rot_y'], pose1['rot_z'])
				p2 = Face(camera2,
							[pose2['x'], pose2['y'], pose2['z']],
							pose2['rot_x'], pose2['rot_y'], pose2['rot_z'])
				p3 = Face(camera3,
							[pose3['x'], pose3['y'], pose3['z']],
							pose3['rot_x'], pose3['rot_y'], pose3['rot_z'])
				p1.convertFaceFromLocalToWorldSpace()
				p2.convertFaceFromLocalToWorldSpace()
				p3.convertFaceFromLocalToWorldSpace()
				# the following is how to access position of the face after converting from local to world space
				# you can test this out by checking two cameras at a time whether a face visible in both cameras is the same position
				rospy.loginfo(p1.position[0])
				rospy.loginfo(p2.position[0])
				rospy.loginfo(p3.position[0])
				# feature computation should be done here


if __name__ == '__main__':
	rospy.init_node('gaze_detector', anonymous=True)
	main()