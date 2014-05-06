#!/usr/bin/python

import time
import datetime
import urllib2
import thread
import speech_to_text as speechText
import audio_processing as audioProcess
import json
import pyaudio
import wave
import os
import threading
import rospy
from std_msgs.msg import String
from pr2_listens.msg import SpeechAPIResponse

path_to_speech_profile = os.path.join(os.getcwd(), 'speech.noise-profile')
base_dir = os.path.join(os.getcwd(), 'files/')
STREAM_TIMEOUT_TIME = 10 #seconds

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

p = pyaudio.PyAudio()
stream = None
stream_access_time = 0
stream_lock = threading.Lock()


def s_callback(in_data, frame_count, time_info, status):
  print "in s_callback"
  return (in_data, pyaudio.paContinue)

def record_audio_server():
  global stream, stream_access_time, stream_lock
  #initialize subscriber with a callback to keep alive messages
  stream_lock.acquire()
  stream = p.open(format=FORMAT, 
    channels=CHANNELS, 
    rate=RATE, 
    input=True,
    output=False,
    frames_per_buffer=CHUNK,
    start=False,
    stream_callback=s_callback)
 
  stream_access_time = stream.get_time()
  stream.start()
  stream_lock.release()

if __name__ == "__main__":
  print "Setting up environment of files and folders"
  record_audio_server()
