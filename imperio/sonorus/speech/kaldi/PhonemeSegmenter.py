import soundfile as sf
from pathlib import Path
from tempfile import gettempdir
import shutil
import numpy as np
import re

from kaldi.decoder import LatticeFasterDecoderOptions
from kaldi.nnet3 import NnetSimpleComputationOptions
from kaldi.asr import NnetLatticeFasterRecognizer
from kaldi.alignment import NnetAligner
from kaldi.fstext import SymbolTable
from kaldi.util.table import SequentialMatrixReader, SequentialWaveReader

from ...utilities.utils import create_random_dir
from ...audio.utils import audio_float2int
from ... import CACHE_DIR

from .create_confs import create_mfcc_conf, create_ivector_extractor_conf
from .utils import (
    LIBRISPEECH_TGSMALL_URL,
    MODEL_ITEM_FILENAMES,
    download_model,
    get_model_item_filepaths,
)

ROUND_DECIMAL = 6


def simplify_phoneme(phoneme):
    r"""remove digit as count id of phonemes and further sub-categories followed 
    by _ e.g. AE1_I will be simplified to AE"""
    return re.sub(r"(\d+|_.$)", "", phoneme)


class PhonemeSegmenter(object):
    def __init__(
        self,
        model_rxfilename,
        graph_rxfilename,
        symbols_filename,
        tree_rxfilename,
        lexicon_rxfilename,
        disambig_rxfilename,
        phoneme_file,
        mfcc_conf,
        ivec_conf,
        decoder_opts=dict(beam=13, max_active=7000),
        decodable_opts=dict(
            acoustic_scale=1.0, frame_subsampling_factor=3, frames_per_chunk=150
        ),
        simplify_phoneme=simplify_phoneme,  # set to None if not desired
        as_dict=True,  # return phonemes info as dict if True else as tuple
        pos2key=(
            "name",
            "start",
            "duration",
        ),  # keys to be used for dict while repacking
        work_dir=gettempdir(),  # directory to be used for intermediate steps
    ):

        decoder_options = LatticeFasterDecoderOptions()
        decoder_options.beam = decoder_opts["beam"]
        decoder_options.max_active = decoder_opts["max_active"]

        decodable_options = NnetSimpleComputationOptions()
        decodable_options.acoustic_scale = decodable_opts["acoustic_scale"]
        decodable_options.frame_subsampling_factor = decodable_opts[
            "frame_subsampling_factor"
        ]
        decodable_options.frames_per_chunk = decodable_opts["frames_per_chunk"]

        self.asr = NnetLatticeFasterRecognizer.from_files(
            model_rxfilename=model_rxfilename,
            graph_rxfilename=graph_rxfilename,
            symbols_filename=symbols_filename,
            decoder_opts=decoder_options,
            decodable_opts=decodable_options,
        )

        self.aligner = NnetAligner.from_files(
            model_rxfilename=model_rxfilename,
            tree_rxfilename=tree_rxfilename,
            lexicon_rxfilename=lexicon_rxfilename,
            symbols_filename=symbols_filename,
            disambig_rxfilename=disambig_rxfilename,
            decodable_opts=decodable_options,
        )

        self.phoneme_table = SymbolTable.read_text(phoneme_file)
        self.mfcc_conf = mfcc_conf
        self.ivec_conf = ivec_conf

        self.simplify_phoneme = simplify_phoneme

        self.as_dict = as_dict
        self.pos2key = pos2key

        self.work_dir = work_dir

    @classmethod
    def from_dir(
        cls,
        model_dir,
        model_item_filenames=MODEL_ITEM_FILENAMES,
        mfcc_conf_kwargs=dict(),  # empty dict, defaults will be used
        ivec_conf_kwargs=dict(),  # empty dict, defaults will be used
        decoder_opts=dict(beam=13, max_active=7000),
        decodable_opts=dict(
            acoustic_scale=1.0, frame_subsampling_factor=3, frames_per_chunk=150
        ),
    ):

        model_item_filepaths = get_model_item_filepaths(
            model_dir, model_item_filenames=model_item_filenames,
        )

        mfcc_conf = create_mfcc_conf(model_dir, **mfcc_conf_kwargs)
        ivec_conf = create_ivector_extractor_conf(model_dir, **ivec_conf_kwargs)

        args = [
            str(model_item_filepaths.get(k))
            for k in (
                "model_rxfilename",
                "graph_rxfilename",
                "symbols_filename",
                "tree_rxfilename",
                "lexicon_rxfilename",
                "disambig_rxfilename",
                "phoneme_file",
            )
        ] + [mfcc_conf, ivec_conf]

        return cls(*args, decoder_opts=decoder_opts, decodable_opts=decodable_opts,)

    @classmethod
    def from_url(
        cls,
        url=LIBRISPEECH_TGSMALL_URL,
        model_item_filenames=MODEL_ITEM_FILENAMES,
        cache_dir=CACHE_DIR,
        force_download=False,
        mfcc_conf_kwargs=dict(),  # empty dict, defaults will be used
        ivec_conf_kwargs=dict(),  # empty dict, defaults will be used
        decoder_opts=dict(beam=13, max_active=7000),
        decodable_opts=dict(
            acoustic_scale=1.0, frame_subsampling_factor=3, frames_per_chunk=150
        ),
    ):

        return cls.from_dir(
            model_dir=download_model(url, cache_dir, force_download),
            model_item_filenames=model_item_filenames,
            mfcc_conf_kwargs=mfcc_conf_kwargs,
            ivec_conf_kwargs=ivec_conf_kwargs,
            decoder_opts=decoder_opts,
            decodable_opts=decodable_opts,
        )

    def segment(self, audio, sample_rate=22050, time_level=True, clean_up=True):

        temp_dir = create_random_dir(
            work_dir=self.work_dir, prefix="sonorus_phoneme_segmenter"
        )

        audio_file = temp_dir / "audio.wav"
        wav_scp = temp_dir / "wav.scp"
        spk2utt = temp_dir / "spk2utt"

        if len(audio) == 1:  # single row but given as 2D matrix
            audio = audio[0]
        if np.issubdtype(audio.dtype, np.floating):
            audio = audio_float2int(audio)

        sf.write(file=audio_file, data=audio, samplerate=sample_rate)
        with open(wav_scp, "w") as f:
            f.write(f"utt1 {audio_file}")
        with open(spk2utt, "w") as f:
            f.write("utt1 utt1")

        phonemes = self.segment_from_file(wav_scp, spk2utt, time_level=time_level)

        if clean_up:
            shutil.rmtree(temp_dir, ignore_errors=True)

        return phonemes

    def segment_from_file(self, wav_scp, spk2utt, time_level=True):

        audio_durs = dict()

        if time_level:
            try:
                with SequentialWaveReader(f"scp:{wav_scp}") as reader:

                    audio_durs = {
                        key: round(wav.duration, ROUND_DECIMAL) for key, wav in reader
                    }
            except:
                pass

        phonemes = self.phoneme_from_rspecs(
            *self.get_rspecs(wav_scp, spk2utt), audio_durs=audio_durs
        )

        return phonemes

    def phoneme_from_rspecs(self, feats_rspec, ivectors_rspec, audio_durs={}):

        phonemes = dict()

        with SequentialMatrixReader(feats_rspec) as f, SequentialMatrixReader(
            ivectors_rspec
        ) as i:

            for (key, feats), (_, ivectors) in zip(f, i):

                decoded = self.asr.decode((feats, ivectors))

                aligned_phonemes = self.aligner.to_phone_alignment(
                    decoded["alignment"], self.phoneme_table
                )

                aligned_phonemes = self.frame_to_sec(
                    aligned_phonemes, audio_dur=audio_durs.get(key)
                )

                aligned_phonemes = self.repack(self.simplify(aligned_phonemes))

                phonemes[key] = dict(text=decoded["text"], phonemes=aligned_phonemes)

        return phonemes

    def frame_to_sec(self, aligned_phonemes, audio_dur=None):

        # aligned_phonemes are in the form of list of tuples
        # (phoneme, start, duration)
        total_frames = sum(aligned_phonemes[-1][1:])

        def to_sec(frame):
            return round(frame * audio_dur / total_frames, ROUND_DECIMAL)

        if audio_dur is not None:
            aligned_phonemes = [
                (i[0], to_sec(i[1]), to_sec(i[2])) for i in aligned_phonemes
            ]

        return aligned_phonemes

    def simplify(self, aligned_phonemes):
        if self.simplify_phoneme:
            aligned_phonemes = [
                (self.simplify_phoneme(i[0]), i[1], i[2]) for i in aligned_phonemes
            ]
        return aligned_phonemes

    def repack(self, aligned_phonemes):
        if self.as_dict:
            aligned_phonemes = [dict(zip(self.pos2key, i)) for i in aligned_phonemes]
        return aligned_phonemes

    def get_rspecs(self, wav_scp, spk2utt):

        feats_rspec = (
            f"ark:compute-mfcc-feats --config={self.mfcc_conf} scp:{wav_scp} ark:- |"
        )

        ivectors_rspec = (
            f"ark:compute-mfcc-feats --config={self.mfcc_conf} scp:{wav_scp} ark:- |"
            f"ivector-extract-online2 --config={self.ivec_conf} ark:{spk2utt} ark:- ark:- |"
        )

        return feats_rspec, ivectors_rspec
