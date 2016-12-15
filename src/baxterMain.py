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


class baxterMain(object):

    def __init__(self):

        rp.init_node('main_file' , anonymous=True)

        achChannelList = ["baxterImage" , "baxterPosition" , "baxterState"]

        for i in achChannelList:
            subprocess.Popen("ach -1 -C " +  i + " -m 10 -n 3000", shell=True, stdout=subprocess.PIPE)


        toBax = ach.Channel('baxterPosition')
        fromBax = ach.Channel('baxterState')
        imgBax = ach.Channel('baxterImage')

        # Enable baxter robot #
        # subprocess.call("rosrun baxter_tools enable_robot.py -e")
        cmd = ['rosrun', 'baxter_tools', 'enable_robot.py' , '-e']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        cmd = ['rosrun', 'baxter_examples' , 'achTOBaxter.py']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        cmd = ['rosrun', 'baxter_examples' , 'achFROMBaxter.py']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        cmd = ['rosrun', 'baxter_examples' , 'baxterImageProcessing.py']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)

        self._globals()

        rate = rp.Rate(int(rp.get_param('~runRate' , default=100)))
        deltaTheta = float(rp.get_param('~ik_delta_theta' , default=.1))
        pixSize = float(rp.get_param('~pixel_size' , default = .0023))
        focalLength = float(rp.get_param('~focal_size' , default = .0012))
        #
        # dhMatLeft = np.array([[lTheta1 , .27 , .069 , -np.pi/2],
        #                  [lTheta2+np.pi/2 , 0. , 0. , np.pi/2],
        #                   [lTheta3 , 0.102+.262 , 0.69 , -np.pi/2],
        #                   [lTheta4 , 0. , 0. , np.pi/2],
        #                   [lTheta5 , .104+.262 , .01 , -np.pi/2],
        #                   [lTheta6 , 0. , 0. , np.pi/2]])
        #
        # dhMatRight = np.array([[rTheta1 , .27 , .069 , -np.pi/2],
        #                  [rTheta2+np.pi/2 , 0. , 0. , np.pi/2],
        #                   [rTheta3 , 0.102+.262 , 0.69 , -np.pi/2],
        #                   [rTheta4 , 0. , 0. , np.pi/2],
        #                   [rTheta5 , .104+.262 , .01 , -np.pi/2],
        #                   [rTheta6 , 0. , 0. , np.pi/2]])

        toMsg = ROBOT()
        fromMsg = ROBOT()
        img = IMAGE()

        [status , frameSize] = fromBax.get(fromMSG , wait = True , last = True)

        for i in range(BAXTER_NUM_ARM_JOINTS):
            dhMatLeft[i,0] = fromMsg.arm[LEFT_ARM].joint[i].ref
            dhMatRight[i,0] = fromMsg.arm[RIGHT_ARM].joint[i].ref




        while not rp.is_shutdown():

            [statusImg , frameSizeImg] = imgBax.get(img , wait = True , last = True)

            error = np.array([img.vertical[RIGHT_ARM] , img.horizontal[RIGHT_ARM] ,
                              img.vertical[RIGHT_ARM] , img.horizontal[LEFT_ARM]])



            print "image error = ",error



            rate.sleep()

    def _rightArmDH(self,val):
        # rj = right.joint_names() = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

        dhMatRight = np.array([[val[0] , .27 , .069 , -np.pi/2],
                          [val[1]+np.pi/2 , 0. , 0. , np.pi/2],
                          [val[2] , 0.102+.262 , 0.69 , -np.pi/2],
                          [val[3] , 0. , 0. , np.pi/2],
                          [val[4] , .104+.262 , .01 , -np.pi/2],
                          [val[5] , 0. , 0. , np.pi/2]])

    def _leftArmDH(self,val):
        # lj = left.joint_names()  = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

        dhMatRight = np.array([[val[0] , .27 , .069 , -np.pi/2],
                          [val[1]+np.pi/2 , 0. , 0. , np.pi/2],
                          [val[2] , 0.102+.262 , 0.69 , -np.pi/2],
                          [val[3] , 0. , 0. , np.pi/2],
                          [val[4] , .104+.262 , .01 , -np.pi/2],
                          [val[5] , 0. , 0. , np.pi/2]])




    def _globals(self):

        global lTheta1
        global lTheta2
        global lTheta3
        global lTheta4
        global lTheta5
        global lTheta6

        global rTheta1
        global rTheta2
        global rTheta3
        global rTheta4
        global rTheta5
        global rTheta6


if __name__ == "__main__":

    baxterMain()