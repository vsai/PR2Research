#import pyaudio
#import wave


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

def recognizeResponse(srcFilename):
    import urllib2
    import json
    url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
    fname = 'output.flac'
    audio = open(fname, 'rb').read()
    headers={'Content-Type': 'audio/x-flac; rate=16000', 'User-Agent':'Mozilla/5.0'}
    request = urllib2.Request(url, data=audio, headers=headers)
    response = urllib2.urlopen(request)
    r = response.read()
    # return response.read()
    return json.loads(r)


def main():
    import os
    record(4, 'output.wav')
    os.system('sox output.wav output.flac remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither')
    result = recognizeResponse('output.wav')
    #result = recognizeResponse('cleaned.wav')
    print result
    print type(result)
    print result.get('hypotheses')

main()
