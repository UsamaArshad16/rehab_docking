#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import numpy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from math import pow, atan2, sqrt
from sensor_msgs.msg import JointState

target = Pose()
target.position.x = 0
target.position.y = 0

robot_x = robot_y = robot_yaw =  0
marker_pose  = marker_y = marker_yaw =  0
marker_x = 0
marker_id1 = "not_in_frame"
def visual_callback(msg):
    global marker_x, marker_y, marker_yaw, marker_id1
    try:
        marker_x = msg.pose.position.x
        marker_y = msg.pose.position.y
        marker_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        marker_euler = euler_from_quaternion(marker_quaternion)
        marker_yaw = marker_euler[2]+np.pi/2
        marker_id1 = "in_frame"
    except:
        marker_id1 = "not_in_frame"
        print("marker 1 not detected")

def odom_callback(odom):
    global robot_x, robot_y, robot_yaw
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    robot_quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    robot_euler = euler_from_quaternion(robot_quaternion)
    robot_yaw = robot_euler[2]


def getRangeWithAngles(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ Extracts scan data within a specific angle range. Returns two 
    arrays: range data and corresponding angles. Possible to pass two
    ranges which should be returned (e.g. 0-45 and 315-360 in order to
    retrieve 90 degree range in front of robot).
    
    :param data: data of type sensor_msgs/LaserScan
    :param fromIndex: starting index of range to return
    :param toIndex: end index of range to return
    :param fromIndex2: (optional) start index of second range to return
    :param toIndex2: (optional) end index of second range to return
    :return (ranges, angles):   two array containing corresponding ranges
                                and angles in degrees
    """
    resultRanges = []
    resultAngles = []
    for idx, val in enumerate(data.ranges):
        inRange1 = idx >= fromIndex and idx <= toIndex
        inRange2 = toIndex2 and (idx >= fromIndex2 and idx <= toIndex2)
        if inRange1 or inRange2:
            resultRanges.append(val)
            resultAngles.append(idx * data.angle_increment * 180 / math.pi)
    
    return (resultRanges, resultAngles)
    
def getRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ same as getRangeWithAngles(), but only ranges are returned, no angles
    """
    return getRangeWithAngles(data, fromIndex, toIndex, fromIndex2, toIndex2)[0]

def getMinMaxRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ returns minimum and maximum values and corresponding angles within
    a specific range.
    :param data: data of type sensor_msgs/LaserScan
    :param fromIndex: starting index of range to analyze
    :param toIndex: end index of range to analyze
    :param fromIndex2: (optional) start index of second range to analyze
    :param toIndex2: (optional) end index of second range to analyze
    :return (minValue, minAngle, maxValue, maxAngle): minimum and maximum
            values within given ranges with corresponding angles in degrees
    
    """
    (resultRanges, resultAngles) = getRangeWithAngles(data, fromIndex, toIndex, fromIndex2, toIndex2)
    maxValue = max(resultRanges)
    validValues = [i for i in resultRanges if i > 0]
    minValue = min(validValues) if len(validValues) > 0 else 0
    minValueIndex = resultRanges.index(minValue)
    maxValueIndex = resultRanges.index(maxValue)
    minAngle = resultAngles[minValueIndex]
    maxAngle = resultAngles[maxValueIndex]
    
    return (minValue, minAngle, maxValue, maxAngle)
    
def getMinRange(data, fromIndex, toIndex, fromIndex2 = None, toIndex2 = None):
    """ same as getMinMaxRange() but returns only (minValue, minAngle)
    
    """
    result = getMinMaxRange(data, fromIndex, toIndex, fromIndex2, toIndex2)
    return (result[0], result[1])
    

# how much values in front/back are used to calculate front/back distance?
meanCount = 30

# range in degrees for analysing minimum values in front/back
frontBackRange = 90

minValueBack = minValueFront = 0.0
def scanCallback(data):
    global minValueBack, minValueFront
    ##print (minValueBack)
    """ Callback for data coming from topic /scan
    
    :param data: data of type sensor_msgs/LaserScan
    """
    
    (minValue, minAngle, maxValue, maxAngle) = getMinMaxRange(data, 0, len(data.ranges))
    halfRange = frontBackRange / 2
    halfLength = len(data.ranges) / 2
    (minValueFront, minAngleFront) = getMinRange(data, 0, halfRange - 1, 360 - halfRange, 360)
    (minValueBack, minAngleBack) = getMinRange(data, halfLength - halfRange, halfLength + halfRange - 1)
    
    # calculate front/back distances
    length = len(data.ranges)
    halfMeanCount = meanCount / 2
    frontRanges = getRange(data, 0, halfMeanCount - 1, 360 - halfMeanCount, 360)
    backRanges = getRange(data, halfLength - halfMeanCount, halfLength + halfMeanCount - 1)
    backRange = numpy.mean(backRanges)
    frontRange = numpy.mean(frontRanges)


    # msg = ScanAnalyzed()
    # msg.range_min = minValue
    # msg.range_max = maxValue
    # msg.range_front = frontRange
    # msg.range_front_min = minValueFront
    # msg.range_back = backRange
    # msg.range_back_min = minValueBack
    # msg.angle_range_min = minAngle
    # msg.angle_range_max = maxAngle
    # msg.angle_front_min = minAngleFront
    # msg.angle_back_min = minAngleBack

        
twist = Twist()

navigation_status = "False"
def navigation_callback(status, result):
    global navigation_status
    if status == GoalStatus.SUCCEEDED:
        navigation_status = "True"
        rospy.loginfo("Navigation succeeded!")
    else:
        navigation_status = "False"
        rospy.logwarn("Navigation failed with status: " + str(status))

def navigate(x,y,th,w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = th
    goal.target_pose.pose.orientation.w = w
    rospy.loginfo("server is up")
    client.send_goal(goal,done_cb=navigation_callback)
    client.wait_for_result()


def auto_dock_basic():
    if (marker_x > 0.6 and minValueFront > 0.3):
        #print(marker_x, marker_y)
        ##print( marker_y)
        if marker_y>=0.01:
            #print ("Moving +ive")
            twist.linear.x = 0.06
            #twist.linear.x = 0.0
            twist.angular.z = 0.05
            pub.publish(twist)
        elif marker_y<=-0.01:
            #print("Moving -ive")
            twist.linear.x = 0.06
            #twist.linear.x = 0.0
            twist.angular.z = -0.05
            pub.publish(twist)
        else:
            twist.linear.x = 0.06
            twist.angular.z = 0.0    
            pub.publish(twist)
        rospy.sleep(0.1)

    elif marker_x < 0.6:
        #print("j", marker_x)
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
    else:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)



def jointState_callback(data):
    global joint_angles
    joint_angles = data.position

wheel_value_180 = 6.22
def rotate():
    last_wheel_value = joint_angles[0]
    while(abs(joint_angles[0]-last_wheel_value) < wheel_value_180 and not rospy.is_shutdown()):
        twist.angular.z = 0.2
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)


def last_stop():
    while(minValueBack > 0.45 and not rospy.is_shutdown()):
        twist.linear.x = -0.05
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

def dock():
    print("docking called")
    navigate(-1.190, 2.042, 0.315, 0.949) 
    rospy.sleep(1)
    if marker_id1 == "in_frame" and navigation_status == "True":
    #if marker_id1 == "in_frame":
        while(marker_x > 0.6 and not rospy.is_shutdown()):
            auto_dock_basic()
            rospy.sleep(0.1)
        rotate()
        last_stop()
    else:
        print("AR tag is not in the frame")

def undock():
    last_x = robot_x
    last_y = robot_y
    move_distance = 0.3

    while(sqrt(pow((robot_x - last_x), 2) + pow((robot_y - last_y), 2)) < move_distance and not rospy.is_shutdown()):
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

def docking_callback(data):
    message = data.data
    message_list = message.split(",")
    status = message_list[1]
    #print("Status: ", status)
    if status == "dock":
        dock()
    elif status == "undock":
        undock()
        rospy.set_param('/start_navigation', True)

if __name__ == '__main__':
    # Initialize the ROS node with the name 'sound_direction_pan'
    rospy.init_node('dock_undock')
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/aruco_single/pose", PoseStamped, visual_callback)
    odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scanCallback)
    rospy.Subscriber("/cmd_frm_tablet", String, docking_callback)
    rospy.Subscriber("/measured_joint_states", JointState, jointState_callback)
    rospy.spin()
