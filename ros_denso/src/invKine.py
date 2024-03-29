#!/usr/bin/python2.7
import rospy
import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState


class invKine:

    def __init__(self):
        self.thetas = np.zeros(6)
        self.l = [210,75,210]
        self.z_offset = 280
        rospy.init_node('invKine')
        self.pub = rospy.Publisher('/joints_denso', JointState, queue_size=10)
        rospy.Subscriber('/denso_trajectory', Transform, self.command_recieve)


    def command_recieve(self,transform_data):
        rospy.loginfo('Colombia')
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
        L = np.sqrt(w**2 + z**2)
        Lf = np.sqrt(self.l[1]**2 + self.l[2]**2)

        c_phi = (self.l[0]**2 + Lf**2 - L**2) / (2 * self.l[0] * Lf)
        rospy.loginfo(c_phi)
        s_phi = np.sqrt(1 - c_phi**2)
        phi = np.arctan2(s_phi, c_phi)

        c_phi2 = (Lf ** 2 + L ** 2 - self.l[0] ** 2) / (2 * Lf * L)
        rospy.loginfo(c_phi2)
        s_phi2 = np.sqrt(1 - c_phi2**2)
        phi2 = np.arctan2(s_phi2, c_phi2)

        phi1 = np.pi - phi - phi2
        theta_L = np.arctan2(z, w)

        theta_L1 = theta_L + phi1
        alpha = 2 * np.pi - np.arctan2(self.l[2], self.l[1]) - phi

        z_kappa = np.sqrt(Lf ** 2 - (w - self.l[0] * np.sin(theta_L1)) ** 2)
        w_kappa = w - np.sin(theta_L1) * self.l[0]

        kappa = np.arctan2(w_kappa, z_kappa)
        beta = np.arctan2(self.l[2], self.l[1])

        kappa = (np.pi/2) - beta
        true_alpha = np.pi - beta - kappa

        self.thetas[0] = np.arctan2(y, x)
        self.thetas[1] = ((np.pi / 2) - theta_L1) * 0.8582
        self.thetas[2] = alpha - (np.pi / 2)
        self.thetas[3] = 0

        self.thetas[4] = np.pi -(kappa + phi2) - (np.pi/2 - theta_L)
        self.thetas[5] = self.thetas[0] - (2*np.arccos(a))%(np.pi/2)
        joint_state = JointState()
        joint_state.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
        joint_state.position = [0.0,0.0,0.0,0.0,0.0,0.0]
        joint_state.velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
        joint_state.effort = [0.0,0.0,0.0,0.0,0.0,0.0]
        joint_state.position = self.thetas
        rospy.loginfo(joint_state)
        self.pub.publish(joint_state)




if __name__ == '__main__':
    invkine = invKine()
    rospy.spin()

