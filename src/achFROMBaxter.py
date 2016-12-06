#! /usr/bin/env python

"""




"""

import numpy as np
import rospy as rp
from baxterStructure import *
import baxter_interface as bi
import ach
from sensor_msgs.msg import JointState

class ach_FROM_Baxter_Interface(input):

    def __init__(self):
        rp.init_node('from_baxter' , anonymous=True)
        runRate = rp.Rate(rp.get_param('~runRate' , default = '100'))

        jointStateCallback = rp.Subscriber('/robot/joint_states')

        leftArm = bi.Limb("left")
        rightArm = bi.Limb("right")

        s = ach.Channel("baxterState")

        robot = ROBOT()

        leftJointName = leftArm.joint_names()
        rightJointName = rightArm.joint_names()

        while not rp.is_shutdown():

            try:
                robot = self._BaxterDictionary(robot,leftJointName,rightJointName)
                s.put(robot)

            except:
                pass
            runRate.sleep()

    def _BaxterDictionary( self ,  robot , leftJointName , rightJointName):


        for i in range(BAXTER_NUM_ARM_JOINTS):
            robot.arm[LEFT_ARM].joint[i].ref = self.pos(np.where(leftJointName[i] == self.name))
            robot.arm[RIGHT_ARM].joint[i].ref = self.pos(np.where(rightJointName[i] == self.name))

        return robot


    def _jointStateCallback(self,data):

        self.pos = data.position
        self.name = data.name

if __name__ == "__main__":
    ach_TO_Baxter_Interface()