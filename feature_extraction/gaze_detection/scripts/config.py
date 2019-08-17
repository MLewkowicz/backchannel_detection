#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Header, Time, Int32
from gaze_detection.msg import GazeFeatures
from openface_ros.msg import face_info
from rospywrapper import BagSource, TopicSource, TopicSink, BagSink


#configure
def get_configuration():
    # Get topic parameters from each camera
    gaze_topic_camera1 = rospy.get_param('gaze_topic_1', '/camera1/face_info')
    gaze_topic_camera2 = rospy.get_param('gaze_topic_2', '/camera2/face_info')
    gaze_topic_camera3 = rospy.get_param('gaze_topic_3', '/camera3/face_info')

    # get broadcasted start time and window duration
    start_time_topic = rospy.get_param('start_time_topic', '/bc/start_time')
    window_duration_topic = rospy.get_param('window_duration_topic', '/bc/window_duration')

    #create a features topic for publishing the combined nod message
    features_topic = rospy.get_param('features_topic', '/bc/gaze_features')

    # Get source, sink parameters
    src_bag_path = rospy.get_param('source_bag_path', None)
    sink_bag_path = rospy.get_param('sink_bag_path', None)

    # Instantiate sources
    if src_bag_path:
        gaze_src_1 = BagSource(src_bag_path, gaze_topic_camera1)
        gaze_src_2 = BagSource(src_bag_path, gaze_topic_camera2)
        gaze_src_3 = BagSource(src_bag_path, gaze_topic_camera3)

        start_time_src = BagSource(src_bag_path, start_time_topic)
        window_duration_src = BagSource(src_bag_path, window_duration_topic)
    else:
    	gaze_src_1 = TopicSource(gaze_topic_camera1, face_info)
        gaze_src_2 = TopicSource(gaze_topic_camera2, face_info)
        gaze_src_3 = TopicSource(gaze_topic_camera3, face_info)

        start_time_src = TopicSource(start_time_topic, Time)
        window_duration_src = TopicSource(window_duration_topic, Int32)

    # Instantiate sinks
    if sink_bag_path:
        bag = rosbag.Bag(sink_bag_path, 'w')
        features_sink = BagSink(bag, features_topic, GazeFeatures)
    else:
        bag = None
        features_sink = TopicSink(features_topic, GazeFeatures)

    # Get the start time
    rospy.loginfo('Finding start time, window duration.')
    with start_time_src:
        msg, _ = next(start_time_src)
        start_time = msg.data

    # Get the window duration
    with window_duration_src:
        msg, _ = next(window_duration_src)
        window_duration = rospy.Duration(msg.data / 1000.)
    rospy.loginfo('Found start time, window duration.')

    gaze_sources = (gaze_src_1, gaze_src_2, gaze_src_3)
    return (bag, gaze_sources, features_sink, start_time, window_duration)
