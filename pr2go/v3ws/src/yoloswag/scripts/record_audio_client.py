#!/usr/bin/python

import sys
import rospy
from yoloswag.srv import *
from yoloswag.msg import *

cmds = ['left', 'right', 'up', 'down']

def record_audio_client(duration, commands):
	rospy.wait_for_service('record_audio')
	try:
		record_audio = rospy.ServiceProxy('record_audio', RecordAudio)
		resp1 = record_audio(duration, commands)
    rospy.loginfo("Hypothesis: %s", resp1.hypothesis)
		return resp1.hypothesis
	except rospy.ServiceException, e:
    rospy.logerr("Service called failed : %s", e)

def usage():
  rospy.logerr("Input a valid duration <int>")
  return

def handle_text(result_cmds):
  pub = rospy.Publisher('/turtle1/command_velocity', Velocity)
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(0.5)
  r.sleep()
	for i in result_cmds:
		rospy.loginfo(i)
		msg = Velocity()
		if (i == 'left'):
			msg.linear = 0.0
			msg.angular = 2.0
		elif (i == 'right'):
			msg.linear = 0.0
			msg.angular = -2.0
		elif (i == 'up'):
			msg.linear = 2.0
			msg.angular = 0.0
		elif (i == 'down'):
			msg.linear = -2.0
			msg.angular = 0.0
		else:
			msg.linear = 0.0
			msg.angular = 0.0
		pub.publish(msg)
		r.sleep()	
  return result_cmds

if __name__ == "__main__":
	if len(sys.argv) == 2:
		duration = int(sys.argv[1])
	else:
		print usage()
		sys.exit(1)
    	
  rospy.loginfo("Requesting recording duration %s", duration)
	result_cmds = record_audio_client(duration, cmds)

  try:
    handle_text(utterance)
	except rospy.ROSInterruptException: pass

