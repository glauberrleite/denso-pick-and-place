#!/usr/bin/env python

PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)
 
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs import JointTrajectory
import numpy as np

class bridge:
    
    def __init__(self):
        self.msg  = JointTrajectory()
        self.pub = rospy.Publisher('/vp6242/trajectory_controller/command', JointTrajectory,queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        rospy.init_node('bridge')
        rospy.Subscriber("joints", JointState, self.callback)
        
    
    def callback(self,data):
        self.msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.msg.points.positions = data.position
        self.msg.points.velocities = data.velocity
        self.msg.points.effort = data.effort
        self.msg.points.accelerations = np.zeros(6)
        self.pub.publish(self.msg)
        


if __name__ == '__main__':
    bridge = bridge()
    rospy.spin