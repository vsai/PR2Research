#Carnegie Mellon University - Vishalsai Daswani (vhd)
#PR2 Research
#Version 2 implementation of speech recognition

import threading
import logging
import time

#for recording
import pyaudio
import wave

#fixed constants for audio hardware
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

#for API request handling
import urllib2
import json

#for audio processing (with sox lib) and for directory control
import os

baseAudioDir = "v2files/"
baseRawDir = baseAudioDir + "raw/"
baseCleanDir = baseAudioDir + "clean/"
baseFlacDir = baseAudioDir + "flac/"

class tmi(threading.Thread):
	def __init__(self, threadName, recordTime):
		threading.Thread.__init__(self)
		self.threadName = threadName
		self.recordTime = recordTime
		self.rawAudioFile = baseRawDir + self.threadName + ".wav"
		self.cleanAudioFile = baseCleanDir + self.threadName + ".wav"
		self.flacAudioFile = baseFlacDir + self.threadName + ".flac"
		#google speech API fixed URL
		self.apiUrl = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
	
	def recordAudio(self, recordTime, destFile):
		assert (recordTime > 0 and recordTime < 15)
		p = pyaudio.PyAudio()
		stream = p.open(format=FORMAT,
				channels=CHANNELS,
				rate=RATE,
				input=True,
				frames_per_buffer=CHUNK)
		print('* recording')
		frames = []
		for i in xrange(0, int(RATE / CHUNK * recordTime)):
			data = stream.read(CHUNK)
			frames.append(data)
		print('* done recording')

		stream.stop_stream()
		stream.close()
		p.terminate()

		#if you want to do some NUMPY manipulation on the arrays
		#DO IT HEREi

		wf = wave.open(destFile, 'wb')
		wf.setnchannels(CHANNELS)
		wf.setsampwidth(p.get_sample_size(FORMAT))
		wf.setframerate(RATE)
		wf.writeframes(b''.join(frames))
		wf.close()

		return True

	def cleanAudio(self, srcFilename, destFilename):
		cmd = 'sox ' + srcFilename + ' ' + destFilename + ' noisered speech.noise-profile 0.1'
		os.system(cmd)
		return True

	def resampleConvert(self, srcFilename, destFilename):
		cmd = 'sox ' + srcFilename + ' ' + destFilename + ' remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither'
		os.system(cmd)
		return True

	def textToSpeech(self, srcFilename):
		audioFd = open(srcFilename, 'rb')
		audio = audioFd.read()
		audioFd.close()
		headers={'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
		request =  urllib2.Request(self.apiUrl, data=audio, headers=headers)
		response = urllib2.urlopen(request)
		r = response.read()
		jsObj = json.loads(r)
		return jsObj

	def run(self):
		print "Starting", self.threadName
		self.recordAudio(self.recordTime, self.rawAudioFile) #audioin -> raw/<>.wav
		self.cleanAudio(self.rawAudioFile, self.cleanAudioFile) # raw/<>.wav -> clean/<>.wav
		self.resampleConvert(self.cleanAudioFile, self.flacAudioFile) #clean/<>.wav -> flac/<>.flac
		result = self.textToSpeech(self.flacAudioFile) #flac/<>.wav -> APIResponse
		print result
        #TODO: if we got a result, then clean the files


#TODO: DELETE? THIS IS NOT BEING USED
def generateNoiseProfile():
	os.system('sox noise.wav -n trim 0 5 noiseprof speech.noise-profile')

def letsDoThis(count):
	threads = []
	threadsLock = threading.Lock()
	for i in xrange(count):
		myThread = tmi(str(i), 5)
		threadsLock.acquire()
		threads.append(myThread)
		threadsLock.release()
		myThread.start()
		time.sleep(3)
	for t in threads:
		t.join()

if not os.path.isdir(baseRawDir):
    os.makedirs(baseRawDir)
if not os.path.isdir(baseCleanDir):
    os.makedirs(baseCleanDir)
if not os.path.isdir(baseFlacDir):
    os.makedirs(baseFlacDir)

letsDoThis(5)
