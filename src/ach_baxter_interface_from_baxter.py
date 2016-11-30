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



        rp.init_node("baxter_ach_from" , anonymous = True)

        # robot = ROBOT()
        leftArm = bi.Limb("left")
        rightArm = bi.Limb("right")

        s = ach.Channel("baxterState")
        # angles_right = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0}
        # angles_left = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0}


        while True:

            print 'waiting'

            # [statuss , frameSizes] = s.get(robot, wait = True , last = True)

            robot = self._fromBaxterDictionary(  leftArm , rightArm )
            s.put(robot)


            # try:
            #     leftArm.move_to_joint_positions(leftAngle)
            #     rightArm.move_to_joint_positions(rightAngle)
            # except:
            #     print "nothing came through"
            rp.Rate(1).sleep()






    def _fromBaxterDictionary(self, leftArm , rightArm):
        leftJointName = leftArm.joint_names()
        rightJointName = rightArm.joint_names()

        thetaOffsetRight = np.array([np.pi/4.,0.,0.,np.pi/2.,0.,0.,0.])
        thetaOffsetLeft = np.array([-np.pi/4.,0.,0.,np.pi/2.,0.,0.,0.])
        offsetInds = np.array([0,1,2,3,4,5,6])


        robot = ROBOT()

        for i in range(BAXTER_NUM_ARM_JOINTS):
            # S0 = 0
            # S1 = 1
            # E0 = 2
            # E1 = 3
            # W0 = 4
            # W1 = 5
            # W2 = 6

            # leftAngles.update({leftJointName[i] : robot.arm[LEFT_ARM].joint[i].ref + thetaOffsetLeft[i]})
            # rightAngles.update({rightJointName[i] : robot.arm[RIGHT_ARM].joint[i].ref + thetaOffsetRight[i]})

            robot.arm[LEFT_ARM].joint[i].ref = leftArm.joint_angle(leftJointName[i]) - thetaOffsetLeft[i]
            robot.arm[RIGHT_ARM].joint[i].ref = rightArm.joint_angle(rightJointName[i]) - thetaOffsetRight[i]
            print "Left Joint = {} Value = {} Right Joint = {} Value = {}".format(leftJointName[i] , robot.arm[LEFT_ARM].joint[i].ref ,
                                                                                  rightJointName[i] , robot.arm[RIGHT_ARM].joint[i].ref)
        return robot


if __name__=="__main__":
    ach_baxter_interface()