#!/usr/bin/python

import sys
import rospy
from pr2_listens.msg import SpeechAPIResponse#ListenFor
from turtlesim.msg import Velocity
from std_msgs.msg import String

cmds = { \
  'left' : Velocity(linear=0.0, angular=2.0), \
  'right' : Velocity(linear=0.0, angular=-2.0), \
  'up' : Velocity(linear=2.0, angular=0.0), \
  'down' : Velocity(linear=-2.0, angular=0.0), \
  'default' : Velocity(linear=0.0, angular=0.0)
}

def parse_instructions(commands, hypothesis):
  def f(x):
    return (x in commands)
  return filter(f, hypothesis.split())

def callback(publisher_actions, data):
  print "Hypothesis: %s"%(data.hypothesis)
  print "Confidence: %s"%(data.confidence)
  instructions = parse_instructions(cmds.keys(), data.hypothesis)
  
  #for each instruction, do something
  for i in instructions:
    print "SENDING INSTRUCTION: ", i
    msg = cmds.get(i)
    if not msg:
      #command is not in cmds
      msg = cmds.get('default')
    publisher_actions.publish(msg)
    time.sleep(1)

def record_audio_client():
  rospy.init_node('speech_recog_client', anonymous=True)
  #String - 1 attribute: "string record"
  pubRecord = rospy.Publisher('initiateRecord', String)
  #Velocity - 2 attributes: "float32 linear", "float32 angular"
  pubAction = rospy.Publisher('/turtle1/command_velocity', Velocity)
  #APIResponse - 2 attributes: "string hypothesis", "float32 confidence"
  def callbackWrapper(data):
    callback(pubAction, data)
  subWords = rospy.Subscriber('wordsHeard', SpeechAPIResponse, callbackWrapper)
  keep_alive_freq_period = 10 #seconds
  assert(keep_alive_freq_period > 0)
  r = rospy.Rate(1.0/keep_alive_freq_period)
  while not rospy.is_shutdown():
    msg = String(data="YES")
    pubRecord.publish(msg)
    r.sleep()

if __name__ == "__main__":
  try:
    record_audio_client()
  except rospy.ROSInterruptException: pass
