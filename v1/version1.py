#Carnegie Mellon University - Vishalsai Daswani (vhd)
#PR2 Robot research
#Version 1 implementation of voice speech to text

#Works without generating a speech.noiseprofile filtering

def record(seconds, filename):
    import pyaudio
    import wave
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 44100
    #TODO: add support for writing to TEMPFILES
    assert(seconds > 0 and seconds < 15)
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

def textToSpeech(flacFilename, srcFilename):
    import urllib2
    import json
    url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
    audio = open(flacFilename, 'rb').read()
    headers={'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
    request = urllib2.Request(url, data=audio, headers=headers)
    response = urllib2.urlopen(request)
    r = response.read()
    # return response.read()
    return json.loads(r)

def generateNoiseProfile():
    import os
    os.system('sox noise.wav -n trim 0 5 noiseprof speech.noise-profile')
    return

def main():
    import os
    baseDir = "v1files/"
    outputWav = baseDir + "output.wav"
    outputFlac = baseDir + "output.flac"
    cleanedWav = baseDir + "cleaned.wav"
    record(4, outputWav)
    #os.system('sox output.wav cleaned.wav noisered speech.noise-profile 0.1')
    os.system('sox ' + outputWav + ' ' + outputFlac + ' remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither')
    result = textToSpeech(outputFlac, cleanedWav)
    print result
    print result.get('hypotheses')


#record(8, 'noise.wav')
#generateNoiseProfile()
main()

