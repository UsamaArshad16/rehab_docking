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
marker_x = 0
def visual_callback(ar_markers):
    global marker_x, marker_y, marker_yaw
    try:
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
    except:
        llll = 0
        print("marker not in frame")

def odom_callback(odom):
    global robot_x, robot_y, robot_yaw
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    robot_quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    robot_euler = euler_from_quaternion(robot_quaternion)
    robot_yaw = robot_euler[2]

def target_callback(tar):
    global target
    target = tar

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
rospy.init_node('AR_AUTO_DOCKING', anonymous=True)
pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
target_pose_sub = rospy.Subscriber('/target_pose', Pose, target_callback)
rospy.Subscriber('/ar_pose_marker', AlvarMarkers, visual_callback)
odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
rospy.Subscriber("/scan", LaserScan, scanCallback)
rospy.sleep(1)

def navigate(x,y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    ##print(target.position.x, target.position.y)
    goal.target_pose.pose.orientation.z = 0.840404040318
    goal.target_pose.pose.orientation.w = 0.541960375873

    client.send_goal(goal)
    client.wait_for_result()



align = 0
def auto_dock_basic():
    global align
    if (marker_x > 0.4 and minValueFront > 0.3):
        print(marker_x, marker_y)
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

    elif marker_x < 0.4:
        #print("fak", marker_x)
        align = 1
        print("j", marker_x, align)
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
    else:
        twist.linear.x = 0
        twist.angular.z = 0
        #pub.publish(twist)


real_yaw = 0
last_yaw = robot_yaw
d = [0,0]
def absolute_pose():
    global real_yaw
    del d[0]
    d.append(robot_yaw)
    if (robot_yaw<0):
        f =  d[1]-d[0] 
        if (abs(f)>3):
            f = d[1]+d[0]
        real_yaw = real_yaw + f
    else:
        f = d[1]-d[0]
        if (abs(f)>3):
            f = d[1]+d[0]
        real_yaw = real_yaw + f
    return real_yaw

# tolerane = 0.01
# def pose_reach():
#     if absolute_pose()<0:


tolerane = 0.03
def rotate(a):
    #global last_yaw
    last_yaw = robot_yaw
    if a >= 0:
        while((absolute_pose()-last_yaw-a) < -tolerane and not rospy.is_shutdown()): ##sign gets change and it goes again positive, So tolerance is a little big
            twist.angular.z = 0.2
            pub.publish(twist)
            #print((absolute_pose()-last_yaw-a)*180/math.pi, (absolute_pose()-last_yaw)*180/math.pi, absolute_pose()*180/math.pi, "1")
            #print((absolute_pose()-last_yaw-a)*180/math.pi,robot_yaw)
            rospy.sleep(0.1)
        #print("done")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    elif a <= 0:
        while((absolute_pose()-last_yaw-a) > tolerane and not rospy.is_shutdown()):
            twist.angular.z = -0.2
            pub.publish(twist)
            #print((absolute_pose()-last_yaw-a)*180/math.pi, (absolute_pose()-last_yaw)*180/math.pi,absolute_pose()*180/math.pi, "2")
            #print((absolute_pose()-last_yaw-a)*180/math.pi,robot_yaw)
            rospy.sleep(0.1)
        #print("done")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)


def last_stop():
    while(minValueBack > 0.35 and not rospy.is_shutdown()):
        twist.linear.x = -0.05
        pub.publish(twist)
        rospy.sleep(0.1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.signal_shutdown("docking_done")



navigate(0.961, 0.648)
while not rospy.is_shutdown():
    auto_dock_basic()
    if marker_x < 0.4 and align ==1:
         rotate(3.02)
         last_stop()
    rospy.sleep(0.1)