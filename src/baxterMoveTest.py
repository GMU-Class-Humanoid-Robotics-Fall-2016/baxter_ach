
#!/usr/bin/env python
import argparse
import sys
import rospy
import baxter_interface as bi
import ach
from baxterStructure import *
import time

def main():
	print("init node")

	baxterDataRef = baxterStructure.ROBOT()
	r = ach.Channel("baxterRef")
	#s = ach.Channel("baxterState")

	for i in range(BAXTER_NUM_ARM_JOINTS)
            baxterDataRef.arm[LEFT_ARM].joint[i] = 0.0
            baxterDataRef.arm[RIGHT_ARM].joint[i] = 0.0
	
	while True:
		baxterDataRef.currTime = time.time()
		r.put(baxterDataRef)
		

	

if __name__ == '__main__':
	main()
