#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class bridge:
    
    def __init__(self):
        self.msg  = JointTrajectory()
        self.pub = rospy.Publisher('/vp6242/trajectory_controller/command', JointTrajectory,queue_size=10)
        self.sq = 0
        rospy.init_node('bridge')
        rospy.Subscriber("/joints_denso", JointState, self.callback)
        self.msg.header.frame_id = ""
        self.msg.joint_names.append("joint1")
        self.msg.joint_names.append("joint2")
        self.msg.joint_names.append("joint3")
        self.msg.joint_names.append("joint4")
        self.msg.joint_names.append("joint5")
        self.msg.joint_names.append("joint6")
        self.msg.points = []

    
    def callback(self,data):
        a = JointTrajectoryPoint()
        a.positions = data.position
        a.velocities = data.velocity
        a.effort = data.effort
        a.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0]
        a.time_from_start = rospy.Time(1, 0)
        '''
        self.msg.points.positions = data.position
        self.msg.points.velocities = data.velocity
        self.msg.points.effort = data.effort
       '''
        self.msg.points = []
        self.msg.points.append(a)

        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)
        


if __name__ == '__main__':
    bridg = bridge()
    rospy.spin()