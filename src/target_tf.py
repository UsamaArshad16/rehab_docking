#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf2_msgs.msg
import tf2_ros



rospy.init_node('target_tf', anonymous=True)
pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
rospy.sleep(0.5)

def tf_pub():
    #print("publishing frame")
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "ar_marker_1"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "target_loc"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 1.5

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    tfm = tf2_msgs.msg.TFMessage([t])
    pub_tf.publish(tfm)



while not rospy.is_shutdown():
    tf_pub()
    rospy.sleep(0.01)