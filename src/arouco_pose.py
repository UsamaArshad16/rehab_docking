#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose_callback(msg):
    # Extracting the position coordinates from PoseStamped message
    marker_x = msg.pose.position.x
    marker_y = msg.pose.position.y
    marker_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    marker_euler = euler_from_quaternion(marker_quaternion)
    marker_yaw = marker_euler[2]+np.pi/2
    marker_id1 = "in_frame"

    # Print the coordinates
    print(marker_yaw)
def main():
    rospy.init_node('pose_subscriber', anonymous=True)

    # Subscribing to the "/aruco_single/pose" topic
    rospy.Subscriber("/aruco_single/pose", PoseStamped, pose_callback)

    # Spin until node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
