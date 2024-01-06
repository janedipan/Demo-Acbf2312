#! /usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyRob():
    listener_joy = None
    talker_rob = None
    joy_msg = None
    def __init__(self):

        self.theNode = rospy.init_node('joy_stick_manager')
        self.listener_joy = rospy.Subscriber('joy',Joy,self.callback)
        # self.talker_rob = rospy.Publisher('/hunter_1_/cmd_vel',Twist,queue_size=10)
        self.talker_rob = rospy.Publisher('scout1/scout_1_/cmd_vel',Twist,queue_size=10)
        self.rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

    def publish(self):
        if self.joy_msg:
            self.talker_rob.publish(self.joy_msg)

    def callback(self,date):
        cmd = Twist()
        cmd.linear.x = date.axes[1]         #linear speed
        cmd.angular.z = date.axes[0]    #angular speed
        self.joy_msg = cmd  
        # print('speed: %.2f, turn: %.2f'%(cmd.linear.x,cmd.angular.z))


if __name__ == '__main__':
    JoyRob()
