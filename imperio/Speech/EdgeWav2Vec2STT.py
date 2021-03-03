from SpeechRecognizer import AudioStreamer
import numpy as np
import pyaudio

from transformers import Wav2Vec2Tokenizer, Wav2Vec2ForCTC
import torch

# Audio recording parameters
SAMPLE_RATE = 16000
CHUNK = int(SAMPLE_RATE / 10)  # 100ms
PA_FORMAT = pyaudio.paFloat32

tokenizer = Wav2Vec2Tokenizer.from_pretrained("facebook/wav2vec2-base-960h")
model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-base-960h")

def transcribe(audio_input, tokenizer, model):
    input_values = tokenizer(audio_input, return_tensors="pt").input_values
    logits = model(input_values).logits
    predicted_ids = torch.argmax(logits, dim=-1)
    transcription = tokenizer.batch_decode(predicted_ids)[0]
    return transcription

with AudioStreamer(SAMPLE_RATE, CHUNK, PA_FORMAT) as audio_streamer:
    data = np.array([], dtype=np.float32)
    for i, content in enumerate(audio_streamer.stream()):
        data = np.hstack((data, np.frombuffer(content, np.float32)))
        print(i, data.shape)
        if i == 50:
            print(transcribe(data, tokenizer, model))
            data = np.array([], dtype=np.float32)
            break