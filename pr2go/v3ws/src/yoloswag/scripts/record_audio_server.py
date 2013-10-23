#!/usr/bin/python

print "HELLO WORLD FROM RECORDING AUDIO ON SERVER"

import time
import datetime
import urllib2
import json
import pyaudio
import wave
from yoloswag.srv import *
import rospy

def record(seconds, filename):
	CHUNK = 1024
	FORMAT = pyaudio.paInt16
	CHANNELS = 2
	RATE = 44100
	assert (seconds > 0 and seconds < 15)
	p = pyaudio.PyAudio()
	stream = p.open(format=FORMAT, 
			channels=CHANNELS, 
			rate=RATE, 
			input=True, 
			frames_per_buffer=CHUNK)
	
	print('* recording')
	frames = []
	for i in xrange(0, int(RATE / CHUNK * seconds)):
		data = stream.read(CHUNK)
		frames.append(data)
	print('* done recording')
	
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

def txt_to_speech(flac_filename):
	import urllib2
	import json
	url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
	audio = open(flac_filename, 'rb').read()
	headers={'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
	request = urllib2.Request(url, data=audio, headers=headers)
	response = urllib2.urlopen(request)
	r = response.read()
	return json.loads(r)

def dothis(record_time, filename_base):
	if (record_time < 1 or record_time > 15):
		return "ERROR: record time requested out of range [1-15]"
	base_dir = "files/	
	record(record_time, base_dir + filename_base+'_output_.wav')

def handle_record_audio(req):
	print "Returning string of audio recorded"
	print "Requested to record for %s seconds"%(req.d)
	filename_base = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H:%M:%S')
	hyp = dothis(req.d, filename_base)
	return RecordAudioResponse(hyp)	
	#return RecordAudioResponse("helloworld")

def record_audio_server():
	rospy.init_node('record_audio_server')
	s = rospy.Service('record_audio', RecordAudio, handle_record_audio)
	print "Ready to record audio"
	rospy.spin()

if __name__ == "__main__":
	record_audio_server()

