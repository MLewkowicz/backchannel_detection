#!/usr/bin/env python
import rospy
from std_msgs.msg import Time, Int32

def main():
	rospy.init_node('central')

	# Start time
	tp = rospy.Publisher('/bc/start_time', Time, latch=True)
	tp.publish(rospy.Time.from_sec(1560787942.84))

	# Window duration
	#window_duration = rospy.get_param('~duration', 100)
	window_duration = 1
	wp = rospy.Publisher('/bc/window_duration', Int32, latch=True)
	wp.publish(window_duration)

	rospy.spin()

if __name__ == '__main__':
	main()
