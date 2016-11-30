#!/usr/bin/env python
import argparse
import sys
import rospy
import baxter_interface as bi
import ach
#import baxterStructure

def toBaxterDictionary(baxterDataRef):
	lj = left.joint_names()  
	rj = right.joint_names()
	
	

def main():
	print("init node")

	baxterDataRef = baxterStructure.ROBOT()
	r = ach.Channel("baxterRef")
	#s = ach.Channel("baxterState")

	rospy.init_node("baxter_joint_pos_set")
	left  = bi.Limb('left')
	right = bi.Limb('right')
	
	
	rate = rospy.Rate(1000)
	
	 while not rospy.is_shutdown():
		 [statuss, frameSizes] = r.get(baxterDataRef, wait = True, last = True)
		 
		 
	
	#angles_right = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0}
	#angles_left = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0}
	
	
	
	left.move_to_joint_positions(angles_left)
	right.move_to_joint_positions(angles_right)
	
	

	print left.joint_angle('left_s0')
	print right.joint_angle('right_e1')

	

if __name__ == '__main__':
	main()
