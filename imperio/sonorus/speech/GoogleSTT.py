from google.cloud import speech

try:
    # additional explicit imports for compatibility with
    # google-cloud-speech=1.3.2 for python2
    from google.cloud.speech import enums
    from google.cloud.speech import types
except ImportError:
    pass

from pyaudio import paInt16
from ..audio import AudioInputStreamer


def default_callback(text, **kwargs):
    if kwargs.get("reset"):
        print(text)


class GoogleSTT(object):
    def __init__(
        self, lang="en-US", audio_streamer=None, punctuation=True, context_phrases=[],
    ):

        self._lang = lang

        self._audio_streamer = (
            AudioInputStreamer(pa_format=paInt16)
            if audio_streamer is None
            else audio_streamer
        )

        self._punctuation = punctuation
        self._phrases = context_phrases

    def _get_speech_client_and_config(self):
        client = speech.SpeechClient()

        recognition_config = dict(
            sample_rate_hertz=self._audio_streamer.processing_rate,
            language_code=self._lang,
            max_alternatives=1,
            enable_automatic_punctuation=self._punctuation,
            use_enhanced=True,
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
            encoding=encoding, speech_contexts=[speech_contexts], **recognition_config
        )

        streaming_config = StreamingRecognitionConfig(
            config=config, interim_results=True
        )

        return client, streaming_config

    def streaming_transcribe(self, callback=default_callback, **callback_kwargs):

        client, streaming_config = self._get_speech_client_and_config()

        with self._audio_streamer as audio_streamer:

            try:
                StreamingRecognizeRequest = speech.StreamingRecognizeRequest
            except AttributeError:
                # Python2 and speech 1.3.2 compatibility
                StreamingRecognizeRequest = types.StreamingRecognizeRequest

            requests = (
                StreamingRecognizeRequest(audio_content=stream)
                for stream in audio_streamer.stream()
            )

            responses = client.streaming_recognize(streaming_config, requests)

            # Now, put the transcription responses to use.
            self._handle_recognized(responses, callback, **callback_kwargs)

    def _handle_recognized(
        self, recognized, callback=default_callback, **callback_kwargs
    ):

        for response in recognized:
            if response.results and (
                len(response.results) > 1 or response.results[0].is_final
            ):

                result = response.results[0]
                if result.alternatives:
                    text = result.alternatives[0].transcript
                    confidence = int(100 * result.alternatives[0].confidence)
                    callback_kwargs["reset"] = result.is_final

                    callback(text, **callback_kwargs)
