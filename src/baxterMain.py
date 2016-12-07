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



class baxterMain(object):

    def __init__(self):

        self._globals()
        rate = rp.Rate(int(rp.get_param('~runRate' , default=100)))
        dTheta = float(rp.get_param('~ik_delta_theta' , default=.1))

        dhMat = np.array([[theta1 , .27 , .069 , -np.pi/2],
                         [theta2+np.pi/2 , 0. , 0. , np.pi/2],
                          [theta3 , 0.102+.262 , 0.69 , -np.pi/2],
                          [theta4 , 0. , 0. , np.pi/2],
                          [theta5 , .104+.262 , .01 , -np.pi/2],
                          [theta6 , 0. , 0. , np.pi/2]])

        toMsg = ROBOT()
        fromMsg = ROBOT()

        toBax = ach.Channel('baxterPosition')
        fromBax = ach.Channel('baxterState')
        imgBax = ach.Channel('baxterImage')

        while not rp.is_shutdown():



            rate.sleep()


    def _globals(self):
        global theta1
        global theta2
        global theta3
        global theta4
        global theta5
        global theta6