from copy import deepcopy
import random
import numpy as np
from .VisemesPublisher import (
    VisemesPublisher,
    VISEMES_TOPIC,
    set_control,
)
from .utils import get_phoneme2viseme


class PhonemesPublisher(VisemesPublisher):
    def __init__(
        self,
        phoneme2viseme=None,
        phoneme_key="name",
        sil="Sil",
        viseme_params={},
        default_viseme_params={},
        topic=VISEMES_TOPIC,
        pub_queue_size=10,
        set_control=set_control,
    ):

        super(PhonemesPublisher, self).__init__(
            topic=topic, pub_queue_size=pub_queue_size, set_control=set_control,
        )

        if phoneme2viseme is None:
            phoneme2viseme, sil, _ = get_phoneme2viseme(drop_params=("duration",))

        self.phoneme2viseme = phoneme2viseme
        self.phoneme_key = phoneme_key
        self.sil = sil

        self.viseme_params = viseme_params
        self.default_params = default_viseme_params

    def random(self, duration, chunk=0.12, seed=None):
        random_gen = random.Random(seed)
        phonemes = list(self.phoneme2viseme.keys())

        phonemes = [
            {
                self.phoneme_key: random_gen.choice(phonemes),
                "start": j,
                "duration": round(min(chunk, duration - i * chunk), 3),
            }
            for i, j in enumerate(np.arange(0, duration, chunk))
        ]

        if len(phonemes) > 1:
            # make the last phoneme always a silence phoneme
            phonemes[-1][self.phoneme_key] = self.sil

        return phonemes

    def publish(self, phonemes):
        self.visemes_publish(self.to_visemes(phonemes))

    def to_visemes(self, phonemes):

        visemes = deepcopy(phonemes)
        for v in visemes:
            viseme = self.phoneme2viseme.get(v[self.phoneme_key], self.sil)
            v[self.phoneme_key] = viseme
            v.update(deepcopy(self.viseme_params.get(viseme, self.default_params)))

        return visemes
