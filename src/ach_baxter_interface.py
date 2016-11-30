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

        robot = baxterStructure.ROBOT()
        leftArm = bi.limb("left")
        rightArm = bi.limb("right")

        r = ach.Channel("baxterRef")
        # angles_right = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0}
        # angles_left = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0}


        while not rp.is_shutdown():


            [statuss , frameSizes] = r.get(robot, wait = True , last = True)

            leftAngle , rightAngle = self._toBaxterDictionary( leftArm , rightArm )



            left.move_to_joint_positions(angleLeft)
            right.move_to_joint_positions(anglesRight)

            rp.Rate(1).sleep()






    def _toBaxterDictionary(self,robot , leftArm , rightArm):
        leftJointName = leftArm.joint_names()
        rightJointName = rightArm.joint_names()
        leftAngles = {}
        rightAngles = {}

        for i in range(BAXTER_NUM_ARM_JOINTS):
            leftAngles.update({leftJointName[i] : robot.arm[LEFT_ARM].joint[i]})
            rightAngles.update({rightJointName[i] : robot.arm[RIGHT_ARM].joint[i]})



        return leftAngles , rightAngles