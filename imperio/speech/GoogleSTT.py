from google.cloud import speech

try:
    # additional explicit imports for compatibility with
    # google-cloud-speech=1.3.2 for python2
    from google.cloud.speech import enums
    from google.cloud.speech import types
except ImportError:
    pass

import pyaudio
from .AudioStreamer import AudioStreamer
from .BaseSTT import BaseSTT, SAMPLE_RATE, CHUNK

PA_FORMAT = pyaudio.paInt16


class GoogleSTT(BaseSTT):
    def __init__(
        self,
        rate=SAMPLE_RATE,
        chunk=CHUNK,
        pa_format=PA_FORMAT,
        lang="en-US",
        text_batcher=None,
        text_batch_processor=None,
    ):

        super(GoogleSTT, self).__init__(
            rate, chunk, pa_format, lang, text_batcher, text_batch_processor
        )

        self._punctuation = True
        self._phrases = (
            getattr(self._text_batch_processor, "context_phrases", [])
            + getattr(self._text_batch_processor, "action_phrases", [])
            + getattr(self._text_batch_processor, "expression_phrases", [])
            + getattr(self._text_batch_processor, "animation_phrases", [])
        )

    def _get_speech_client_and_config(self):
        client = speech.SpeechClient()

        recognition_config = dict(
            sample_rate_hertz=self._rate,
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

    def streaming_transcribe(self):

        client, streaming_config = self._get_speech_client_and_config()

        with AudioStreamer(self._rate, self._chunk, self._pa_format) as audio_streamer:

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
            self._handle_recognized(responses)

    def _handle_recognized(self, recognized):

        for response in recognized:
            if response.results and (
                len(response.results) > 1 or response.results[0].is_final
            ):

                result = response.results[0]
                if result.alternatives:
                    text = result.alternatives[0].transcript
                    confidence = int(100 * result.alternatives[0].confidence)

                    self._process_text(text, reset=result.is_final)
