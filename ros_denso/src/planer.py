#!/usr/bin/python2.7
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform

class planner:
    def __init__(self):
        self.goal = Transform()
        self.trajectory = []
        rospy.init_node('planner')
        self.is_planning = False
        self.pub = rospy.Publisher('/denso_trajectory', Transform, queue_size=10)
        rospy.Subscriber('/denso_cube_tf', TransformStamped, self.recieve_tf)

    def recieve_tf(self, msg):
        if(not self.is_planning):
            self.is_planning = True
            self.goal = msg.transform
            self.goal.translation.z = self.goal.translation.z - 80
            self.plan()
            self.move()

    def plan(self):
           if(self.is_planning):
                msg = Transform()
                msg.rotation = [0.0, np.sqrt(2)/2, 0.0 ,np.sqrt(2)/2]
                msg.translation = [self.goal.translation.x, self.goal.translation.y, 80+self.goal.translation.z]
                self.trajectory.append(msg)
                msg.rotation[0.0, 1, 0.0, 0.0]
                self.trajectory.append(msg)
                dz = (100-self.goal.translation.z)/10
                for i in range(1,11):
                    msg.translation.z = msg.translation.z-dz
                    self.trajectory.append(msg)
                #Todo Gripper
                for i in range(1,11):
                    msg.translation.z = msg.translation.z+dz
                    self.trajectory.append(msg)

    def move(self):
        while len(self.trajectory) > 0:
            mov = self.trajectory.pop()
            rospy.sleep(1)
            self.pub.publish(mov)



if __name__ == '__main__':
    pl = planner()
    rospy.spin()