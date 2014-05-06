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

FORMAT = pyaudio.paInt16
#CHANNELS = 2
CHANNELS = 1
RATE = 44100
CHUNK = RATE* 4 #5

p = pyaudio.PyAudio()
stream = None
stream_access_time = 0
stream_lock = threading.Lock()

def get_filenames():
  filename_base = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
  output = os.path.join(base_dir, '%s_output.wav'%(filename_base))
  clean = os.path.join(base_dir, '%s_cleaned.wav'%(filename_base))
  flac = os.path.join(base_dir, '%s.flac'%(filename_base))
  return (output, clean, flac)

def record_wave(output_filename, frames):
  wf = wave.open(output_filename, 'wb')
  wf.setnchannels(CHANNELS)
  wf.setsampwidth(p.get_sample_size(FORMAT))
  wf.setframerate(RATE)
  wf.writeframes(b''.join(frames))
  wf.close()


def timeout_stream():
  global stream_lock, stream, stream_access_time
  print "STARTED TIMEOUT_STREAM THREAD"
  while True:
    stream_lock.acquire()
    if (time.time() - stream_access_time > STREAM_TIMEOUT_TIME):
      stream.stop_stream()
    stream_lock.release()
    time.sleep(STREAM_TIMEOUT_TIME)

def keep_alive_record_callback(data):
  global stream_lock, stream, stream_access_time
  print "RECEIVED PING FROM CLIENT", data.data
  stream_lock.acquire()
  stream_access_time = time.time()
  if not stream.is_active():
    stream.start_stream()
  stream_lock.release()

def record_audio_server():
  global stream, stream_access_time, stream_lock
  #initialize subscriber with a callback to keep alive messages
  rospy.init_node('speech_recog_server')
  #String - 1 attribute : "string record"
  subRecord = rospy.Subscriber('initiateRecord', String, keep_alive_record_callback)
  #APIResponse - 2 attributes: "string hypothesis", "float32 confidence"
  pubWords = rospy.Publisher('wordsHeard', SpeechAPIResponse)
  def s_callback(in_data, frame_count, time_info, status):
    print "s_callback"
    (output, clean, flac) = get_filenames()
    record_wave(output, in_data)
    audioProcess.clean_audio(output, clean, flac)
    r = speechText.speech_to_txt(flac)
    hypotheses = r.get('hypotheses', [])
    utterance = ''
    confidence = 0
    if (len(hypotheses) > 0):
      utterance = (hypotheses[0]).get('utterance')
      confidence = float((hypotheses[0]).get('confidence'))
    print "Utterance: %s"%(utterance)
    msg = SpeechAPIResponse(hypothesis=utterance, confidence = confidence)
    pubWords.publish(msg)
    #publish r to 'wordsHeard'
    return (in_data, pyaudio.paContinue)
    
  stream_lock.acquire()
  stream = p.open(format=FORMAT, 
    channels=CHANNELS, 
    rate=RATE, 
    input=True,
    output=False,
    frames_per_buffer=CHUNK,
    start=False,
    stream_callback=s_callback)
 
  stream_access_time = time.time()
  stream_lock.release()
  thread.start_new_thread(timeout_stream, ())
  rospy.spin()

if __name__ == "__main__":
  print "Setting up environment of files and folders"
  if not os.path.exists(path_to_speech_profile):
    audioProcess.generateNoiseProfile()
  if not os.path.exists(base_dir):
    os.makedirs(base_dir)

  print "Initializing rospy audio server"
  record_audio_server()
