#!/usr/bin/python
#import roslib; roslib.load_mainifest('yoloswag')

import sys

import rospy
from yoloswag.srv import *

def record_audio_client(duration):
	rospy.wait_for_service('record_audio')
	try:
		record_audio = rospy.ServiceProxy('record_audio', RecordAudio)
		resp1 = record_audio(duration)
		return resp1.hypothesis
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	return "Input a valid duration <int>"

if __name__ == "__main__":
	if len(sys.argv) == 2:
		duration = int(sys.argv[1])
	else:
		print usage()
		sys.exit(1)

	
	print "Requesting recording duration %s"%(duration)
	print "Result: %s"%(record_audio_client(duration))
