from pygsr import Pygsr
speech = Pygsr()
speech.record(5) #duration in seconds
phrase, complete_response = speech.speech_to_text('en_US')
print phase
