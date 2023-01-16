#!/usr/bin/env python

import rospy
from rehab_msgs.msg import SensorState, WheelsCmdStamped
from geometry_msgs.msg import Twist

twist = Twist()
p_s = WheelsCmdStamped()
pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
pub_single = rospy.Publisher('/wheel_cmd_velocities', WheelsCmdStamped, queue_size=10)

# ir_data = {3: [3,0,0,0], 5: [0,5,0,0], 7: [0,0,7,0], 11: [0,0,0,11],
# 8: [3,5,0,0], 10: [3,0,7,0], 14: [3,0,0,11], 12: [0,5,7,0], 16: [0,5,0,11], 18: [0,0,7,11],
# 15: [3,5,7,0], 19: [3,5,0,11], 21: [3,0,7,11], 23: [0,5,7,11],
# 26: [3,5,7,11],
# 0: [0,0,0,0]}

ir_data = {3: [1,0,0,0], 5: [0,1,0,0], 7: [0,0,1,0], 11: [0,0,0,1],
8: [1,1,0,0], 10: [1,0,1,0], 14: [1,0,0,1], 12: [0,1,1,0], 16: [0,1,0,1], 18: [0,0,1,1],
15: [1,1,1,0], 19: [1,1,0,1], 21: [1,0,1,1], 23: [0,1,1,1],
26: [1,1,1,1],
0: [0,0,0,0]}

def move(s):
    twist.linear.x = s
    twist.angular.z=0
    pub.publish(twist)

def rotate(s):
    twist.linear.x = 0
    twist.angular.z= s
    pub.publish(twist)
        
def docking_algo():
    print(FL, FR, BL, BR)

#Back
    # if FL==0 and FR==0 and BL==1 and BR==1: #Both front IRs 1
    #     print("move forward")
    #     move(-0.05)

    # elif FL==0 and FR==0 and BL==1 and BR==0:
    #     print("rotate_cw")
    #     rotate(-0.1)

    # elif FL==0 and FR==0 and BL==0 and BR==1:
    #     print("rotate_ccw")
    #     rotate(0.1)


#front
    if FL==1 and FR==1 and BL==0 and BR==0: #Both front IRs 1
        print("move forward")
        move(0.05)

    elif FL==1 and FR==0 and BL==0 and BR==0:
        print("rotate_cw")
        rotate(0.1)

    elif FL==0 and FR==1 and BL==0 and BR==0:
        print("rotate_ccw")
        rotate(-0.1)

    # if FL==0 and FR==1 and BL==0 and BR==0: 
    #     print("anticlockiwise rotation")
    #     while not rospy.is_shutdown():
    #         print(FL, FR, BL, BR)
    #         p_s.wheels_cmd.angular_velocities.joint=[0.3, 0.0]
    #         pub_single.publish(p_s)
    #         rospy.sleep(0.01)
    #         if FL == 1:
    #             p_s.wheels_cmd.angular_velocities.joint=[0.0, 0.0]
    #             pub_single.publish(p_s)
    #             rospy.sleep(0.01)
    #             break

    # if FL==1 and FR==0 and BL==0 and BR==0: 
    #     print("clockiwise rotation")
    #     while not rospy.is_shutdown():
    #         print(FL, FR, BL, BR)
    #         p_s.wheels_cmd.angular_velocities.joint=[0.0, 0.3]
    #         pub_single.publish(p_s)
    #         rospy.sleep(0.01)
    #         if FR == 1:
    #             p_s.wheels_cmd.angular_velocities.joint=[0.0, 0.0]
    #             pub_single.publish(p_s)
    #             rospy.sleep(0.01)
    #             break

def callback(msg):
    global FL, FR, BL, BR
    FL = ir_data[msg.bumper][0]
    FR = ir_data[msg.bumper][1]
    BL = ir_data[msg.bumper][2]
    BR = ir_data[msg.bumper][3]
    docking_algo()
    #print(FL, FR, BL, BR)
    #print(ir_data[msg.bumper])

def main():
    rospy.init_node('dockingStationNode', anonymous=True)
    print("Node started")
    rospy.Subscriber("/docking_station", SensorState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
