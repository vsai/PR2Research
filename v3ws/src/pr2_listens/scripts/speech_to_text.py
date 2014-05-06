#!/usr/bin/python

import urllib2
import json

def speech_to_txt(flac_filename):
  print "CONTACTING GOOGLE SPEECH API"
  url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
  audio = open(flac_filename, 'rb').read()
  headers = {'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
  request = urllib2.Request(url, data=audio, headers=headers)
  response = urllib2.urlopen(request)
  r = response.read()
  print "Response: %s"%(r) 
  return json.loads(r)

if __name__ == "__main__":
  if (len(sys.argv) < 2):
    print "Not enough arguments"
    exit(-1)
  filename = sys.argv[1]
  #TODO: should add a check that filename is if extension .flac
  r = speech_to_text(filename)
  #return r
