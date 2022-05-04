#!/usr/bin/env python

import rospy, tf, tf2_ros, geometry_msgs.msg, nav_msgs.msg

def callback(data, args):
	broad = tf2_ros.TransformBroadcaster()
	tf = geometry_msgs.msg.TransformStamped()

	tf.header.stamp = rospy.Time.now()
	tf.header.frame_id = args[0]
	tf.child_frame_id = args[1]
	tf.transform.translation = data.pose.pose.position
	tf.transform.rotation = data.pose.pose.orientation	
	broad.sendTransform(tf)

if __name__ == "__main__":
	rospy.init_node("odometrytransfomation")
	odomInput = rospy.get_param("~odom_input")
	tfOutput  = rospy.get_param("~tf_output")
	rospy.Subscriber(odomInput, nav_msgs.msg.Odometry, callback, [odomInput, tfOutput])
	rospy.spin()
