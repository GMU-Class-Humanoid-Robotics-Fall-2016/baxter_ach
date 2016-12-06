
#!/usr/bin/env python
import argparse
import sys
import rospy as rp
import baxter_interface as bi
import ach
from baxterStructure import *
import time

def main():
	rp.init_node('baxterTest', anonymous=True)

	baxterDataRef = ROBOT()
	r = ach.Channel("baxterRef")
	#s = ach.Channel("baxterState")

	for i in range(BAXTER_NUM_ARM_JOINTS):
		baxterDataRef.arm[LEFT_ARM].joint[i].ref = 0.0
		baxterDataRef.arm[RIGHT_ARM].joint[i].ref = 0.0

	while True:
		baxterDataRef.currTime = time.time()
		r.put(baxterDataRef)

		time.sleep(1)


if __name__ == '__main__':
	main()
