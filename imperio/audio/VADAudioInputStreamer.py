import numpy as np
import pyaudio
import webrtcvad

from .AudioInputStreamer import AudioInputStreamer
from .utils import audio_float2int, vad_collector


class VADAudioInputStreamer(AudioInputStreamer):
    """Filter & segment audio with Voice Activity Detection (VAD)."""

    SAMPLE_RATE = AudioInputStreamer.SAMPLE_RATE
    CHANNELS = AudioInputStreamer.CHANNELS
    BLOCKS_PER_SECOND = AudioInputStreamer.BLOCKS_PER_SECOND
    PA_FORMAT = AudioInputStreamer.PA_FORMAT
    FILE_CHUNK = AudioInputStreamer.FILE_CHUNK
    FMT2TYPE = AudioInputStreamer.FMT2TYPE

    def __init__(
        self,
        aggressiveness=2,
        padding_dur_ms=300,
        act_inact_ratio=0.8,
        sample_rate=SAMPLE_RATE,
        blocks_per_second=BLOCKS_PER_SECOND,
        pa_format=PA_FORMAT,
        channels=CHANNELS,
        processing_rate=SAMPLE_RATE,
        device=None,
        file_path=None,
        file_chunk=FILE_CHUNK,
        callback=None,
    ):
        r"""aggressiveness is an integer between 0 and 3, 0 being the least 
        aggressive about filtering out non-speech while 3 being the most."""

        super().__init__(
            sample_rate=sample_rate,
            blocks_per_second=blocks_per_second,
            pa_format=pa_format,
            channels=channels,
            processing_rate=processing_rate,
            device=device,
            file_path=file_path,
            file_chunk=file_chunk,
            callback=callback,
        )

        self.vad = webrtcvad.Vad(aggressiveness)
        self.act_inact_ratio = act_inact_ratio

        frame_dur_ms = (self.processing_block_size * 1000) // self.processing_rate
        self.num_padding_frames = padding_dur_ms // frame_dur_ms

        self._dtype_conv_fn = (
            audio_float2int
            if self.pa_format == pyaudio.paFloat32
            else lambda frame, float_type, int_type: frame
        )

    def stream(self):

        yield from vad_collector(
            self._stream(),
            vad=self.vad,
            sample_rate=self.processing_rate,
            num_padding_frames=self.num_padding_frames,
            act_inact_ratio=self.act_inact_ratio,
            frame_dtype_conv_fn=self._dtype_conv_fn,
        )
