#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import numpy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import tf2_msgs.msg
import math
import tf2_ros
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from sensor_msgs.msg import LaserScan

target = Pose()
target.position.x = 0
target.position.y = 0

robot_x = robot_y = robot_yaw =  0
marker_pose  = marker_y = marker_yaw =  0
marker_x = 1
def visual_callback(ar_markers):
    global marker_x, marker_y, marker_yaw
    try:
        #print(ar_markers.markers[0].pose.pose.position.x)
        for i in range(len(ar_markers.markers)):
            if (ar_markers.markers[i-1].id == 1):
                marker_x = ar_markers.markers[i-1].pose.pose.position.x
                marker_y = ar_markers.markers[i-1].pose.pose.position.y
                marker_quaternion = [ar_markers.markers[i-1].pose.pose.orientation.x, ar_markers.markers[i-1].pose.pose.orientation.y, ar_markers.markers[i-1].pose.pose.orientation.z, ar_markers.markers[i-1].pose.pose.orientation.w]
                marker_euler = euler_from_quaternion(marker_quaternion)
                marker_yaw = marker_euler[2]+np.pi/2
            else:
                #print("marker 1 not detected")
                x = 0


        print(marker_x,marker_y,marker_yaw)
        # for mkr in ar_markers.markers:
        #     marker_pose = mkr.pose
        #     marker_pose.header = mkr.header
        #     #print(marker_pose.id)
        # marker_x = marker_pose.pose.position.x
        # #print(marker_x)
        # marker_y = marker_pose.pose.position.y
        # marker_quaternion = [marker_pose.pose.orientation.x, marker_pose.pose.orientation.y, marker_pose.pose.orientation.z, marker_pose.pose.orientation.w]
        # marker_euler = euler_from_quaternion(marker_quaternion)
        # marker_yaw = marker_euler[2]+np.pi/2
    except:
        llll = 0
        print("marker not in frame")

rospy.init_node('AR_AUTO_', anonymous=True)
rospy.Subscriber('/ar_pose_marker', AlvarMarkers, visual_callback)
rospy.sleep(1)


while not rospy.is_shutdown():
    rospy.sleep(0.1)