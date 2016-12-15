#! /usr/bin/env python

"""




"""



import numpy as np


def getJacobian(deltaTheta, current , desired , dh):

    out = np.zeros([np.size(current),3])

