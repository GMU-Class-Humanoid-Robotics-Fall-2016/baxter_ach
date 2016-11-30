#! /usr/bin/env python

"""




"""

import numpy as np
import rospy as rp
from baxterStructure import *
import baxter_interface as bi
import ach

class ach_baxter_interface(object):

    def __init__(self):



        rp.init_node("baxter_ach" , anonymous = True)

        robot = ROBOT()
        leftArm = bi.Limb("left")
        rightArm = bi.Limb("right")

        r = ach.Channel("baxterRef")
        # angles_right = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0}
        # angles_left = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0}


        while True:

            print 'waiting'

            [statuss , frameSizes] = r.get(robot, wait = True , last = True)

            leftAngle , rightAngle = self._toBaxterDictionary( robot , leftArm , rightArm )

            try:
                leftArm.move_to_joint_positions(leftAngle)
                rightArm.move_to_joint_positions(rightAngle)
            except:
                print "nothing came through"
            rp.Rate(1).sleep()






    def _toBaxterDictionary(self,robot , leftArm , rightArm):
        leftJointName = leftArm.joint_names()
        rightJointName = rightArm.joint_names()
        leftAngles = {}
        rightAngles = {}

        thetaOffset = np.array([0.,0.,0.,0.,0.,0.,0.])
        offsetInds = np.array([0,1,2,3,4,5,6])




        for i in range(BAXTER_NUM_ARM_JOINTS):
            # S0 = 0
            # S1 = 1
            # E0 = 2
            # E1 = 3
            # W0 = 4
            # W1 = 5
            # W2 = 6

            leftAngles.update({leftJointName[i] : robot.arm[LEFT_ARM].joint[i].ref})
            rightAngles.update({rightJointName[i] : robot.arm[RIGHT_ARM].joint[i].ref})



        return leftAngles , rightAngles


if __name__=="__main__":
    ach_baxter_interface()