#!/usr/bin/python2.7
import rospy
import numpy as np
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform

class planner:
    def __init__(self):
        self.goal = Transform()
        self.trajectory = []
        rospy.init_node('planner')
        self.is_planning = False
        self.pub = rospy.Publisher('/denso_trajectory', Transform, queue_size=10)
        rospy.Subscriber('/denso_cube_tf', tfMessage, self.recieve_tf)

    def recieve_tf(self, msg):
        if(not self.is_planning):
            rospy.logwarn('Entrei Carai')
            self.is_planning = True

            self.goal = msg.transforms[0].transform
            self.goal.translation.z = self.goal.translation.z - 0.08
            self.plan()
            self.move()

    def plan(self):
           if(self.is_planning):
                msg = Transform()
                '''msg.rotation.x = 0.0
                msg.rotation.y = np.sqrt(2)/2
                msg.rotation.z = 0.0
                msg.rotation.w = np.sqrt(2)/2'''
                msg.rotation = self.goal.rotation
                msg.translation.x = self.goal.translation.x*1000
                msg.translation.y = self.goal.translation.y*1000
                msg.translation.z = 80+(self.goal.translation.z*1000)
                #rospy.loginfo(m)
                self.trajectory.append(msg)
                msg = Transform()
                msg.translation.x = self.goal.translation.x * 1000
                msg.translation.y = self.goal.translation.y * 1000
                msg.translation.z = 80 + (self.goal.translation.z * 1000)
                msg.rotation = self.goal.rotation
                self.trajectory.append(msg)
                dz = (100-(self.goal.translation.z*1000))/4
                lastz = msg.translation.z
                for i in range(1,5):
                    msg = Transform()
                    msg.rotation = self.goal.rotation
                    msg.translation.x = self.goal.translation.x * 1000
                    msg.translation.y = self.goal.translation.y * 1000
                    msg.translation.z = lastz-dz
                    self.trajectory.append(msg)
                    lastz = msg.translation.z
                #Todo Gripper
                for i in range(1,5):
                    msg.rotation = self.goal.rotation
                    msg.translation.x = self.goal.translation.x * 1000
                    msg.translation.y = self.goal.translation.y * 1000
                    msg.translation.z = lastz + dz
                    self.trajectory.append(msg)
                    lastz = msg.translation.z

    def move(self):
        while len(self.trajectory) > 0:
            mov = self.trajectory.pop(0)
            rospy.sleep(0.5)
            self.pub.publish(mov)
            rospy.loginfo(mov)
        self.is_planning = False


if __name__ == '__main__':
    pl = planner()
    rospy.spin()