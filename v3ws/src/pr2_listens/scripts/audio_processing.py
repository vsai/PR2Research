#!/usr/bin/python

import pyaudio
import wave
import os
import rospy

path_to_speech_profile = 'speech.noise-profile'
base_dir = 'files/'

def generateNoiseProfile():
  noise_path = 'noise.wav'
  rospy.loginfo("** Start generating noise profile - Path: %s", path_to_speech_profile)
  record_blocking(5, noise_path)
  #Generate a noise profile based on the noise audio just recorded (trim to 5 seconds)
  os.system('sox ' + noise_path + ' -n trim 0 5 noiseprof ' + path_to_speech_profile)
  rospy.loginfo("** Done generating noise profile")	
    
  rospy.logdebug("Cleaning files")
  try:
    os.remove(noise_path)
  except:
    rospy.logerr("Not able to delete noise audio file: %s", noise_path)  
  return

def record_blocking(seconds, filename):
  CHUNK = 44100 #1024
  FORMAT = pyaudio.paInt16
  CHANNELS = 1 # 2
  RATE = 44100
  assert (seconds > 0 and seconds < 15)
  p = pyaudio.PyAudio()
  stream = p.open(format=FORMAT, 
		channels=CHANNELS, 
		rate=RATE, 
		input=True, 
		frames_per_buffer=CHUNK)
	
  rospy.loginfo("** Recording")	
  frames = []
  for i in xrange(0, int(RATE / CHUNK * seconds)):
    data = stream.read(CHUNK)
    frames.append(data)
  rospy.loginfo("** Done Recording")
	
  stream.stop_stream()
  stream.close()
  p.terminate()

  wf = wave.open(filename, 'wb')
  wf.setnchannels(CHANNELS)
  wf.setsampwidth(p.get_sample_size(FORMAT))
  wf.setframerate(RATE)
  wf.writeframes(b''.join(frames))
  wf.close()
	
  return True

def clean_audio(output, clean, flac):
  #Process the output file and clean it and provide a  resampled flac file
  os.system('sox ' + output + ' ' + clean + ' noisered ' + path_to_speech_profile + ' 0.3')
  #os.system('sox {0} {1} noisered {2} 0.2'%(output, clean, path_to_speech_profile))
  #os.system('sox ' + clean + ' ' + flac + ' remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither')
  os.system('sox ' + clean + ' ' + flac + ' highpass 22 gain -2 rate 16k')
