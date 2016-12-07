#! /usr/bin/env python


"""





"""

import numpy as np
import rospy as rp
from baxterStructure import *
import baxter_interface as bi
import ach

class ach_TO_Baxter_Interface(input):

    def __init__(self):
        rp.init_node('to_baxter' , anonymous=True)
        runRate = rp.Rate(int(rp.get_param('~runRate' , default=100)))
        leftArm = bi.Limb("left")
        rightArm = bi.Limb("right")
        robot = ROBOT()
        p = ach.Channel("baxterPosition")

        while not rp.is_shutdown():
            [status,frameSizes] = p.get(robot , wait=True , last=True)

            leftAngle , rightAngle = self._BaxterDictionary( leftArm , rightArm , robot )

            try:
                leftArm.move_to_joint_positions(leftAngle)
                rightArm.move_to_joint_positions(rightAngle)

            except:
                pass

            runRate.sleep()

    def _BaxterDictionary( self , leftArm , rightArm , robot ):

        leftJointName = leftArm.joint_names()
        rightJointName = rightArm.joint_names()

        leftAngles = []
        rightAngles = []

        for i in range(BAXTER_NUM_ARM_JOINTS):
            leftAngles.update({leftJointName[i]: robot.arm[LEFT_ARM].joint[i].ref})
            rightAngles.update({rightJointName[i]: robot.arm[RIGHT_ARM].joint[i].ref})

        return leftAngles , rightAngles


if __name__ == "__main__":
    ach_TO_Baxter_Interface()