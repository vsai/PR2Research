#!/usr/bin/python

import time
import datetime
import urllib2
import json
import pyaudio
import wave
import os
from yoloswag.srv import *
import rospy


path_to_speech_profile = 'speech.noise-profile'
base_dir = 'files/'

def generateNoiseProfile():
	noise_path = 'noise.wav'
    rospy.loginfo("** Start generating noise profile - Path: %s", path_to_speech_profile)
	record(5, noise_path)
    #Generate a noise profile based on the noise audio just recorded (trim to 5 seconds)
    os.system('sox ' + noise_path + ' -n trim 0 5 noiseprof ' + path_to_speech_profile)
    rospy.loginfo("** Done generating noise profile")	
    
    rospy.debug("Cleaning files")
    try:
        os.remove(noise_path)
    except:
        rospy.logerr("Not able to delete noise audio file: %s", noise_path)
    
    return

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

def speech_to_txt(flac_filename):
    rospy.logdebug("Submitting speech file to API to receive response")	
    url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
    audio = open(flac_filename, 'rb').read()
    headers={'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
	request = urllib2.Request(url, data=audio, headers=headers)
    response = urllib2.urlopen(request)
	r = response.read()
    rospy.logdebug("Received response from API: %s", r)
	return json.loads(r)

def clean_audio(output, clean, flac):
    #Process the output file and clean it and provide a  resampled flac file
    os.system('sox ' + output + ' ' + clean + ' noisered ' + path_to_speech_profile + ' 0.3 remix - norm -3 highpass 22 gain -3 norm -3 dither')
	#os.system('sox ' + clean + ' ' + flac + ' remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither')
	os.system('sox ' + clean + ' ' + flac + ' rate 16k')	
	

def dothis(record_time, cmds, filename_base):
	if (record_time < 1 or record_time > 15):
		return "ERROR: record time requested out of range [1-15]"
	output = base_dir + filename_base + '_output.wav'
	clean = base_dir + filename_base + '_cleaned.wav'
	flac = base_dir + filename_base + '.flac'
	record(record_time, output)
    
    clean_audio(output, clean, flac)	
    
    result = speech_to_txt(flac)
	hypotheses = result.get('hypotheses')
	print hypotheses	
	utterance = ''	
	if (len(hypotheses) > 0):
		utterance = (hypotheses[0]).get('utterance')
	print "Utterance: %s"%(utterance)	
	def f(x):
		return (x in cmds)
	words = utterance.split()
	result = filter(f, words)
	return result	

def handle_record_audio(req):
    rospy.loginfo("Requested to record for %s seconds", req.d)	
	filename_base = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
	hyp = dothis(req.d, req.cmds, filename_base)
    
    return RecordAudioResponse(hyp)	

def record_audio_server():
	rospy.init_node('record_audio_server', log_level=rospy.DEBUG)
	s = rospy.Service('record_audio', RecordAudio, handle_record_audio)
	rospy.loginfo("Record_audio service setup - Ready to record audio")
	rospy.spin()

if __name__ == "__main__":
    rospy.loginfo("Setting up environment of files and folders")
    if not os.path.exists(path_to_speech_profile):
		generateNoiseProfile()
	if not os.path.exists(base_dir):
		os.makedirs(base_dir)

    rospy.loginfo("Initializing rospy audio server")
    record_audio_server()

