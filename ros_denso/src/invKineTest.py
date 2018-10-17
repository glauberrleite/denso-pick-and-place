#!/usr/bin/python2.7
import rospy
import numpy as np
import sys
import signal
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class invKine:

    def __init__(self):
        self.thetas = np.zeros(6)
        self.l = [210,75,210]
        #self.l = [335, 88, 210]
        rospy.init_node('invKineTest')
        self.z_offset = 280
        self.pub = rospy.Publisher('/vp6242/trajectory_controller/command', JointTrajectory, queue_size=10)
        self.publish()

    def calculate_joints(self,x,y,z, display_angles = True, display_misc = True):
        w = np.sqrt(x**2 + y**2)
        L = np.sqrt(w**2 + z**2)
        Lf = np.sqrt(self.l[1] ** 2 + self.l[2] ** 2)

        c_phi = ((self.l[0]**2 + Lf**2) - L**2) / (2 * self.l[0] * Lf)
        s_phi = np.sqrt(1 - c_phi**2)
        phi = np.arctan2(s_phi, c_phi)

        c_phi2 = (Lf**2 + L**2 - self.l[0]**2) / (2 * Lf * L)
        s_phi2 = np.sqrt(1 - c_phi2**2)
        phi2 = np.arctan2(s_phi2, c_phi2)

        phi1 = np.pi - phi - phi2
        theta_L = np.arctan2(z, w)

        theta_L1 = theta_L + phi1
        alpha = 2 * np.pi - np.arctan2(self.l[2], self.l[1]) - phi

        z_kappa = np.sqrt(Lf**2 - (w - self.l[0] * np.sin(theta_L1))**2)
        w_kappa = w - np.sin(theta_L1) * self.l[0]

        kappa = np.arctan2(w_kappa, z_kappa)
        beta = np.arctan2(self.l[2], self.l[1])

        true_alpha = np.pi - beta - kappa

        self.thetas[0] = np.arctan2(y, x)
        self.thetas[1] = ((np.pi / 2) - theta_L1) * 0.8582
        self.thetas[2] = alpha - (np.pi / 2)
        '''
        self.thetas[3] = np.arctan2((2 * c * d - 2 * a * b), (2 * b * d + 2 * a * c))
        self.thetas[4] = np.arctan2((2 * c * d - 2 * a * b),((a ** 2 - b ** 2 - c ** 2 - d ** 2) * np.sin(self.thetas[3])))
        self.thetas[5] = np.arctan2(((1 / (np.tan(np.sin(self.thetas[4])))) * np.sin(self.thetas[3])), (((2 * a * c + 2 * a * d) / (2 * c * d + 2 * a * c)) - (np.cos(self.thetas[3]) / np.sin(self.thetas[4]))))
        '''
        if(display_angles):
            print('------- GOAL -------')
            print(' ')
            print('x = ' + str(x) + ' y = ' + str(y) + ' z = ' + str(z))
            print('------- JOINTS --------')
            print()
            print('Joint angle 1 = ' + str(self.thetas[0]*180/np.pi))
            print('Joint angle 2 = ' + str(self.thetas[1]*180/np.pi))
            print('Joint angle 3 = ' + str(self.thetas[2]*180/np.pi))
        if(display_misc):
            print('-------- MISC ---------')
            print(' ')
            print('L = ' + str(L))
            print('W = ' + str(w))
            print('Lf = ' + str(Lf))
            print('Phi = ' + str(phi*180/np.pi))
            print('Phi1 = ' + str(phi1 * 180 / np.pi))
            print('Phi2 = ' + str(phi2 * 180 / np.pi))
            print('Beta = ' + str(np.arctan2(self.l[2],self.l[1])*180/np.pi))
            print('Alpha = ' + str(alpha*180/np.pi))
            print('W_kappa = ' + str(w_kappa))
            print('-------- MISC ---------')
            print(' ')
            print(' ')
            print(' ')

    def publish(self):
            theta = JointTrajectory()
            thetaPoints = JointTrajectoryPoint()
            thetaPoints.positions = self.thetas
            theta.points = thetaPoints
            self.pub.publish(theta)

    def wait_msg(self):
        while not rospy.is_shutdown():
            try:
                x = input('x = ')
                y = input('y = ')
                z = input('z = ')
                self.calculate_joints(x,y,z)
                self.publish()
                rospy.sleep(2)

            except KeyboardInterrupt:
                print "Shutting down module"

    def signal_handler(self,sig, frame):
        # TODO operacoes de limpeza para quitar o programa
        print('Quita Desgrassa')
        sys.exit(0)

    def choose(self, choice):
        if(choice == 0):
            self.wait_msg()
        elif(choice == 1):
            self.cross_move()
        elif(choice == 2):
            self.square_move()
        else:
            print('Invalide Choice!!')


    def cross_move(self):
        x = [250,250,250,210]
        y = [0,125,-125,0]
        z = [280,420,240,565]
        z = [a - self.z_offset for a in z]
        for i in range(4):
            self.calculate_joints(x[i],y[i],z[i])
            self.publish()
            rospy.sleep(2)

    def square_move(self):
        x = [200, 250, 200, 200,210]
        y = [200, 200, -200, -200,0]
        z = [280, 565, 565, 280,565]
        z = [a - self.z_offset for a in z]
        for i in range(5):
            self.calculate_joints(x[i], y[i], z[i])
            self.publish()
            rospy.sleep(2)


if __name__ == '__main__':
    invkine = invKine()
    signal.signal(signal.SIGINT, invkine.signal_handler)
    while not rospy.is_shutdown():
        print('Choose wisely: ')
        print('0 - For Manual Control')
        print('1 - For Cross Move Test')
        print('2 - For Square Move Test')
        print(' ')
        print(' ')
        choice = input('Make Your Choice Youth !! >>')
        invkine.choose(choice)
    rospy.spin()

