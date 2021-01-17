from google.cloud import speech

try:
    # additional explicit imports for compatibility with 
    # google-cloud-speech=1.3.2 for python2
    from google.cloud.speech import enums
    from google.cloud.speech import types
except ImportError:
    pass

import pyaudio
from six.moves import queue

import re
import string

import rospy
from hr_msgs.msg import TTS

# Audio recording parameters
SAMPLE_RATE = 16000
CHUNK = int(SAMPLE_RATE / 10)  # 100ms

LANGUAGE = "en-US" # a BCP-47 language tag
# LANGUAGE = "en-IN"

CONTEXT_PHRASES = ["Asha", "Hey Asha", "Hey, Asha", "Hi Asha", "Hi, Asha", 
                    "Okay Asha", "Okay, Asha"]
ACTION_PHRASES = ["emergency stop"]

class AudioStreamer(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate=SAMPLE_RATE, chunk=CHUNK):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, 
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def stream(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)

class SpeechRecognizer(object):

    def __init__(self, rate=SAMPLE_RATE, chunk=CHUNK, lang=LANGUAGE, 
                context_phrases=CONTEXT_PHRASES, action_phrases=ACTION_PHRASES):
        
        self._rate = rate
        self._chunk = chunk

        self._lang = lang
        self._context_phrases = context_phrases
        self._action_phrases = action_phrases
        self._phrases = context_phrases + action_phrases
        self._punctuation = True

        speech_topic = rospy.get_param('speech_topic', '/hr/control/speech/say')
        self.pub = rospy.Publisher(speech_topic, TTS, queue_size=10)

    def _get_speech_client_and_config(self):
        client = speech.SpeechClient()
        
        recognition_config = dict(sample_rate_hertz=self._rate,
            language_code=self._lang,
            max_alternatives=1,
            enable_automatic_punctuation=self._punctuation
        )

        try:
            speech_contexts = speech.SpeechContext(phrases=self._phrases)
            encoding = speech.RecognitionConfig.AudioEncoding.LINEAR16
            RecognitionConfig = speech.RecognitionConfig
            StreamingRecognitionConfig = speech.StreamingRecognitionConfig

        except AttributeError:
            # Python2 and speech 1.3.2 compatibility
            speech_contexts = types.SpeechContext(phrases=self._phrases)
            encoding = enums.RecognitionConfig.AudioEncoding.LINEAR16
            RecognitionConfig = types.RecognitionConfig
            StreamingRecognitionConfig = types.StreamingRecognitionConfig

        config = RecognitionConfig(
            encoding=encoding,
            speech_contexts=[speech_contexts],
            **recognition_config
        )
        
        streaming_config = StreamingRecognitionConfig(
            config=config, interim_results=True
        )

        return client, streaming_config

    def transcribe(self):
        client, streaming_config = self._get_speech_client_and_config()

        with AudioStreamer(self._rate, self._chunk) as streamer:

            try:
                StreamingRecognizeRequest = speech.StreamingRecognizeRequest
            except AttributeError:
                # Python2 and speech 1.3.2 compatibility
                StreamingRecognizeRequest = types.StreamingRecognizeRequest

            requests = (StreamingRecognizeRequest(audio_content=content)
                for content in streamer.stream()
            )
            
            responses = client.streaming_recognize(streaming_config, requests)

            # Now, put the transcription responses to use.
            self._handle_responses(responses)

    def _handle_responses(self, responses):
        for response in responses:
            if not response.results:
                continue

            text = []
            confidences = []

            for result in response.results:

                if result.is_final:
                    
                    for alt in result.alternatives:
                        text.append(alt.transcript)
                        confidences.append(alt.confidence)
            
            confidence = 0
            if confidences:
                confidence = sum(confidences) / len(confidences)
                confidence = int(100 * confidence)

            text = ' '.join(text).strip()

            if text:
                context_regex = '|'.join(self._context_phrases)
                context_regex = "^({})[{}]?".format(context_regex, string.punctuation)
                context_phrase = re.search(context_regex, text)
                context_phrase = context_phrase.group(0) if context_phrase else ""

                if context_phrase:
                    text = text[len(context_phrase):].strip()
                    text = "Context phrase, {}, is recognized."\
                            " Intended action for, {}, will be executed."\
                            .format(context_phrase, text)
                    print(text)
                    self.pub.publish(text=text, lang=self._lang)

                else:
                    print('Recognised "{}" in "{}"'.format(text, self._lang))
                    self.pub.publish(text=text, lang="hi-IN")
                        # , request_id=None, agent_id=None)

if __name__ == "__main__":
    rospy.init_node("speech_recognizer")

    while True:
        try:
            SpeechRecognizer().transcribe()
        except Exception as exception:
            print(exception)
    
    # rospy.spin()
