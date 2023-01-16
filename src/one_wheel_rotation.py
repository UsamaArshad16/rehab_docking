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

def rotate():
    print("rotating")
    while (1):
        p_s.wheels_cmd.angular_velocities.joint=[0.0, 0.0]
        pub_single.publish(p_s)
        rospy.sleep(0.01)
    # twist.linear.x = 0.2
    # twist.angular.z = 0.02
    # pub.publish(twist)

def docking_algo():
    print(FL, FR, BL, BR)
    # if FL==0 and FL==0 and BL==1 and BR==1: #Both back IRs 1
    #     print("move backward")
    #     move(-0.05)

    # if FL==0 and FL==0 and BL==1 and BR==0: 
    #     print("snti clockiwise rotation")
    #     rotate()

def callback(msg):
    global FL, FR, BL, BR
    FL = ir_data[msg.bumper][0]
    FR = ir_data[msg.bumper][1]
    BL = ir_data[msg.bumper][2]
    BR = ir_data[msg.bumper][3]
    #rotate()
    docking_algo()
    #print(FL, FR, BL, BR)
    #print(ir_data[msg.bumper])

def main():
    rospy.init_node('dockingStationNode', anonymous=True)
    rospy.Subscriber("/docking_station", SensorState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
