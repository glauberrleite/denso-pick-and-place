#!/usr/bin/python2.7
import rospy
import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState

class invKine:

    def __init__(self):
        self.thetas = np.zeros(6)
        self.l = [0.335,0.088,0.210]
        rospy.init_node('invKine')
        self.pub = rospy.Publisher('joints', JointState, queue_size=10)
        rospy.Subscriber('transform', Transform, self.command_recieve)


    def command_recieve(self,transform_data):

        translation = transform_data.translation
        rotation = transform_data.rotation
        x = translation.x
        y = translation.y
        z = translation.z
        a = rotation.w
        b = rotation.x
        c = rotation.y
        d = rotation.z
        w = np.sqrt(x**2 + y**2)
        self.thetas[0] = np.arctan2(y,x)
        self.thetas[1] = np.arccos((x**2 + y**2 + z**2 + self.l[0]**2 - self.l[1]**2 - self.l[2]**2)/(2*self.l[0]*np.sqrt(x**2 + y**2 + z**2))) + np.atan2(z,np.sqrt(x**2 + y**2))-np.pi/2;
        self.thetas[2] = np.arctan2(z-self.l[0]*np.cos(self.thetas[1]),np.sqrt(x**2 + y**2)+self.l[0]*np.sin(self.thetas[1]))- np.arctan2(self.l[1],self.l[2])-self.thetas[1]-np.pi/2;
        self.thetas[3] = np.arctan2((2*c*d - 2*a*b),(2*b*d + 2*a*c))
        self.thetas[4] = np.arctan2((2*c*d - 2*a*b),((a**2 - b**2 - c**2 - d**2)*np.sin(self.thetas[3])))
        self.thetas[5] = np.arctan2( ((1/(np.tan( np.sin(self.thetas[4])) ))*np.sin(self.thetas[3])),(((2*a*c + 2*a*d)/(2*c*d + 2*a*c)) - (np.cos(self.thetas[3])/np.sin(self.thetas[4]))) )
        joint_state = JointState()
        joint_state.name.append('Denso')
        joint_state.position.append(0.0)
        joint_state.velocity.append(0.0)
        joint_state.position = self.thetas
        self.pub.publish(self.thetas)




if __name__ == '__main__':
    invkine = invKine()
    rospy.spin()
