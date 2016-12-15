#! /usr/bin/env python

"""




"""

import numpy as np
from dhMatrix import dhMatrixCalculation


def getJacobian(deltaTheta , dh):
    out = np.zeros([3 , np.shape(dh)[0]])


    for i in range(np.shape(dh)[0]):
        newCurrent = dh
        newCurrent[i,0] += deltaTheta

        output = dhMatrixCalculation(newCurrent)

        out[:,i] = (np.dot(output[0:3,0:3], output[0:3,3])) / deltaTheta

    return out

def inverseKinematics(deltaTheta , dh , error , goal , step=1):

    count = 0
    thetas = np.zeros(shape=[7,10])
    while error > .05:

        jacobian = getJacobian(deltaTheta , dh)
        jacobian = np.linalg.pinv(jacobian)
        nStep = nextStep(step , error , goal , dh)

        thetas[:,count] = np.dot(jacobian , nStep)

        for i in range(7):
            dh[i,0] += thetas[i,count]


        count += 1
        if count > 9:
            print "ten iterations achieved"

            returnThis = []

            break


    return dh[:,0] , thetas

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

    dhMatLeft = np.array([[self.robot.arm[LEFT_ARM].joint[0].ref, .27, .069, -np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[1].ref + np.pi / 2, 0., 0., np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[2].ref, 0.102 + .262, 0.69, -np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[3].ref, 0., 0., np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[4].ref, .104 + .262, .01, -np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[5].ref, 0., 0., np.pi / 2],
                          [self.robot.arm[LEFT_ARM].joint[6].ref, .2295, 0., 0.]])

    return dhMatLeft


#

def nextStep(stepSize , distance , goal,dh):
    current = dhMatrixCalculation(dh)
    current = np.dot(current[0:3,0:3],current[0:3,3])
    dx = (goal[0] - current[0]) * stepSize / distance
    dy = (goal[1] - current[1]) * stepSize / distance
    dz = (goal[2] - current[2]) * stepSize / distance

    return np.transpose(np.array([dx , dy , dz]))