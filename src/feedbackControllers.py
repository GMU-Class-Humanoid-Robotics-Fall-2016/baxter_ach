#! /usr/bin/env python

"""






"""

import numpy as np


class PIDStructure(object):

    def __init__(self):

        self.prevError = []
        self.curError = []
        self.integration = []
        self.pk = []
        self.pd = []
        self.pi = []
        self.time = []



def PID(input):

    gain = input.curError * input.pk
    derivative = ((input.prevError - input.curError) / input.time) * input.pd
    integralReturn = input.integration + input.curError
    integral = integralReturn * input.pi

    input.prevError = input.curError
    input.integration = integralReturn

    return gain+derivative+integral , input


