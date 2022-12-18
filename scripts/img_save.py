#!/usr/bin/env python3
import rospy
import r200 as r200
from time import sleep
import argparse

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Image count')
	parser.add_argument('count', type=int, help='image file save name')
	args = parser.parse_args()
	try:
			R200 = r200.R200_output()
	except rospy.ROSInterruptException:
			pass


	R200.click_depth_image()
	R200.click_rgb_image()
	R200.img_count = args.count
	sleep(2)
	R200.save_depth()
	R200.save_rgb()