#! /usr/bin/env python

"""




"""

import numpy as np
from dhMatrix import dhMatrixCalculation

def inverseKinematics(dhMat,error):

    print 'asdf'




# lSideJacobian = np.linalg.pinv(lSideJacobian)
# rSideJacobian = np.linalg.pinv(rSideJacobian)
#
# leftNextStep = self._nextStep(step, leftError, state, s, 'L', leftGoal[goal,])
# rightNextStep = self._nextStep(step, rightError, state, s, 'R', rightGoal[goal,])
#
# lThetaUpdate = np.dot(lSideJacobian, leftNextStep)
# rThetaUpdate = np.dot(rSideJacobian, rightNextStep)

def _nextStep(self,step,distance,state,s,side,goal):
    current = self._getFK(0,False,side,state,s)
    dx = (goal[0] - current[0]) * step / distance
    dy = (goal[1] - current[1]) * step / distance
    dz = (goal[2] - current[2]) * step / distance

    return np.transpose(np.array([dx,dy,dz]))