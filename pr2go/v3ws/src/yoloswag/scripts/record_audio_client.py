#!/usr/bin/python
#import roslib; roslib.load_mainifest('yoloswag')

import sys

import rospy
from yoloswag.srv import *
from yoloswag.msg import *

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

def parse_text(utterance):
    	words = utterance.split()
    	def f(x):
        	possible_cmds = ['left', 'right', 'up', 'down']
        	return (x in possible_cmds)
    	return filter(f, words)

def handle_text(utterance):
    	cmds = parse_text(utterance)
    	print cmds
   	pub = rospy.Publisher('/turtle1/command_velocity', Velocity)
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(0.5)
	for i in cmds:
		rospy.loginfo(i)
		pub.publish(i)
		r.sleep() 
    	return cmds

if __name__ == "__main__":
	if len(sys.argv) == 2:
		duration = int(sys.argv[1])
	else:
		print usage()
		sys.exit(1)

	print "Requesting recording duration %s"%(duration)
	utterance = record_audio_client(duration)
    	print "Result: %s"%(utterance)
    	handle_text(utterance)
