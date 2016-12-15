#! /usr/bin/env python

"""

image channel = 'baxterImage'
position to baxter = 'baxterPosition'
position from baxter = 'baxterState'



"""

import rospy as rp
import numpy as np
from feedbackControllers import *
import ach
from baxterStructure import *
from inverseKinematics import inverseKinematics
import os
import sys
import subprocess
from dhMatrix import dhMatrixCalculation
import time

import sys, select, termios, tty


class baxterMain(object):

    def __init__(self):

        rp.init_node('main_file' , anonymous=True)

        achChannelList = ["baxterImage" , "baxterPosition" , "baxterState"]

        for i in achChannelList:
            subprocess.Popen("ach -1 -C " +  i + " -m 10 -n 3000", shell=True, stdout=subprocess.PIPE)

        self.robot = ROBOT()
        self.toBax = ach.Channel('baxterPosition')
        self.fromBax = ach.Channel('baxterState')
        self.imgBax = ach.Channel('baxterImage')

        # Enable baxter robot #
        # subprocess.call("rosrun baxter_tools enable_robot.py -e")
        cmd = ['rosrun', 'baxter_tools', 'enable_robot.py' , '-e']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        # cmd = ['rosrun', 'baxter_examples' , 'achTOBaxter.py']
        # p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        # cmd = ['rosrun', 'baxter_examples' , 'achFROMBaxter.py']
        # p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        # cmd = ['rosrun', 'baxter_examples' , 'baxterImageProcessing.py']
        # p = subprocess.Popen(cmd, stdout=subprocess.PIPE)

        self.step = .1
        self.deltaTheta = .1

        self.moveBindings = {
                'i':(.05 , 0. , 0.),
                'j':(0., -.05 , 0.),
                'k':(-.05 , 0. , 0.),
                'l':(0. , .05 , 0.),
                'y':(0.,0.,.05),
                't':(0.,0.,-.05)}

        self.resetBindings = {'a':(0)}

        print "HERE"

        # self._runSim()

        while (1):

            time.sleep(.1)



    def _runSim(self):
        [statuss , frameSize] = self.fromBax.get(self.robot,wait=False,last=False)



        print "HERE AGAIN"

        while (1):
            key = self._getKey()
            if key in self.moveBindings.keys():
                x = self.moveBindings[key][0]
                y = self.moveBindings[key][1]
                z = self.moveBindings[key][2]

                self._doIt(x,y,z)

            elif key in self.resetBindings.keys():
                for i in range(7):
                    self.robot.arm[LEFT_ARM].joint[i].ref = 0.
                print "resetting"

                self.toBax.put(self.robot)

            elif (key == '\x03'):
                break

            else:
                x = 0
                y = 0
                z = 0

            # print "adding" , x , y , z



    def _doIt(self,x,y,z):
        print x , y , z
        left = self._leftArmDH()
        current = dhMatrixCalculation(left)
        current = np.dot(current[0:3,0:3],current[0:3,3])
        # print current
        goal = np.array([current[0] + x , current[1] + y , current[2] + z])
        print "current = {} , goal = {}".format(current, goal)
        error = np.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2 + (goal[2] - current[2])**2)

        newDH, out = inverseKinematics(self.deltaTheta, left, error, goal,self.step)
        inds = np.sum(out,axis=0)
        keep = np.squeeze(np.where(inds != 0))
            # for i in range(np.size(keep)):
            #     for j in range(7):
            #         self.robot.arm[LEFT_ARM].joint[j].ref += out[]
            #
            #         self.toBax.put(self.robot)
            #
        # for j in range(7):
        #     self.robot.arm[LEFT_ARM].joint[j].ref += newDH[]
        #
        #     self.toBax.put(self.robot)
    def _leftArmDH(self):
        # rj = right.joint_names() = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

        # S0 = 0
        # S1 = 1
        # E0 = 2
        # E1 = 3
        # W0 = 4
        # W1 = 5
        # W2 = 6

        [statuss, frameSize] = self.fromBax.get(self.robot, wait=False, last=False)


        dhMatLeft = np.array([[self.robot.arm[LEFT_ARM].joint[0].ref , .27 , .069 , -np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[1].ref+np.pi/2 , 0. , 0. , np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[2].ref , 0.102+.262 , 0.69 , -np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[3].ref , 0. , 0. , np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[4].ref , .104+.262 , .01 , -np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[5].ref , 0. , 0. , np.pi/2],
                          [self.robot.arm[LEFT_ARM].joint[6].ref , .2295 , 0. , 0.]])

        return dhMatLeft

    #
    def _getKey(self):
        settings = termios.tcgetattr(sys.stdin)

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key



if __name__ == "__main__":
    baxterMain()