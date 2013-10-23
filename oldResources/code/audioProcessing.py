import os

noiseFilepath = 'noise.wav'
noiseProfilePath = 'speech.noise-profile'
inputToProcess = 'output.wav'
outputToProcess = 'cleaned.wav'
outputFlac = 'final.flac'

def generateNoiseProfile():
    os.system('sox ' + noiseFilepath + ' -n trim 0 5 noiseprof ' + noiseProfilePath)
    return

def removeNoise():
    os.system('sox ' + inputToProcess + ' ' + outputToProcess + ' noisered ' + noiseProfilePath + ' 0.1')
    return

def resampleConvert():
    os.system('sox ' + outputToProcess + ' ' + outputFlac + ' remix - norm -3 highpass 22 gain -3 rate 16k norm -3 dither')
    return

generateNoiseProfile()
removeNoise()
resampleConvert()
