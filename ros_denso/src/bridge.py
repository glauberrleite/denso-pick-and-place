#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import numpy as np

class bridge:
    
    def __init__(self):
        self.msg  = JointTrajectory()
        self.pub = rospy.Publisher('/vp6242/trajectory_controller/command', JointTrajectory,queue_size=10)
        rospy.init_node('bridge')
        rospy.Subscriber("/joints", JointState, self.callback)
        
    
    def callback(self,data):
        rospy.loginfo(data)
        self.msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.msg.points.positions = data.position
        self.msg.points.velocities = data.velocity
        self.msg.points.effort = data.effort
        self.msg.points.accelerations = np.zeros(6)
        self.pub.publish(self.msg)
        


if __name__ == '__main__':
    bridg = bridge()
    rospy.spin()