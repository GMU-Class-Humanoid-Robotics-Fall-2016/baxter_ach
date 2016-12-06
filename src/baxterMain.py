#! /usr/bin/env python

"""

image channel = 'baxterImage'
position to baxter = 'baxterPosition'
position from baxter = 'baxterState'



"""

import rospy as rp
import numpy as np
from feedbackControllers import *

class baxterMain(object):

    def __init__(self):

        self._globals()



        dhMat = np.array([[theta1 , .27 , .069 , -np.pi/2],
                         [theta2+np.pi/2 , 0. , 0. , np.pi/2],
                          [theta3 , 0.102+.262 , 0.69 , -np.pi/2],
                          [theta4 , 0. , 0. , np.pi/2],
                          [theta5 , .104+.262 , .01 , -np.pi/2],
                          [theta6 , 0. , 0. , np.pi/2]])


    def _globals(self):
        global theta1
        global theta2
        global theta3
        global theta4
        global theta5
        global theta6