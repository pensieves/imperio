from pathlib import Path
import torch
from fairseq import tasks
from omegaconf import open_dict
from fairseq.dataclass.utils import convert_namespace_to_omegaconf


from flashlight.lib.text.dictionary import create_word_dict, load_words
from flashlight.lib.text.decoder import (
    LexiconDecoderOptions,
    LexiconFreeDecoderOptions,
    KenLM,
    SmearingMode,
    Trie,
    LexiconDecoder,
    LexiconFreeDecoder,
)

from .W2lDecoder import W2lDecoder
from .FairseqLM import FairseqLM
from ..utils import to_device


class W2lFairseqLMDecoder(W2lDecoder):
    def __init__(
        self,
        token_dict,
        lexicon,
        lang_model,
        gpu_idx=None,
        nbest=1,
        criterion="ctc",
        unit_lm=False,
        beam=5,
        beam_size_token=100,
        beam_threshold=25,
        lm_weight=1.5,
        word_weight=0.5,
        unk_weight=float("-inf"),
        sil_weight=0.5,
        asg_transitions=None,
        max_replabel=None,
    ):

        super().__init__(token_dict, nbest, criterion, asg_transitions, max_replabel)

        self.unit_lm = unit_lm

        self.lexicon = load_words(lexicon) if lexicon else None
        self.idx_to_wrd = {}

        checkpoint = torch.load(lang_model, map_location="cpu")

        if "cfg" in checkpoint and checkpoint["cfg"] is not None:
            lm_args = checkpoint["cfg"]
        else:
            lm_args = convert_namespace_to_omegaconf(checkpoint["args"])

        with open_dict(lm_args.task):
            lm_args.task.data = str(Path(lang_model).parent)

        task = tasks.setup_task(lm_args.task)
        model = task.build_model(lm_args.model)
        model.load_state_dict(checkpoint["model"], strict=False)

        model, device = to_device(model, gpu_idx, for_eval=True, return_device=True)
        model.make_generation_fast_()

        self.trie = Trie(self.vocab_size, self.silence)

        self.word_dict = task.dictionary
        self.unk_word = self.word_dict.unk()
        self.lm = FairseqLM(self.word_dict, model, model_device=device)

        if self.lexicon:

            start_state = self.lm.start(False)

            for i, (word, spellings) in enumerate(self.lexicon.items()):

                if self.unit_lm:
                    word_idx = i
                    self.idx_to_wrd[i] = word
                    score = 0

                else:
                    word_idx = self.word_dict.index(word)
                    _, score = self.lm.score(start_state, word_idx, no_cache=True)

                for spelling in spellings:
                    spelling_idxs = [token_dict.index(token) for token in spelling]

                    assert (
                        token_dict.unk() not in spelling_idxs
                    ), f"{spelling} {spelling_idxs}"

                    self.trie.insert(spelling_idxs, word_idx, score)

            self.trie.smear(SmearingMode.MAX)

            self.decoder_opts = LexiconDecoderOptions(
                beam_size=beam,
                beam_size_token=(
                    beam_size_token if beam_size_token else len(token_dict)
                ),
                beam_threshold=beam_threshold,
                lm_weight=lm_weight,
                word_score=word_weight,
                unk_score=unk_weight,
                sil_score=sil_weight,
                log_add=False,
                criterion_type=self.criterion_type,
            )

            self.decoder = LexiconDecoder(
                self.decoder_opts,
                self.trie,
                self.lm,
                self.silence,
                self.blank,
                self.unk_word,
                [],
                self.unit_lm,
            )

        else:
            assert (
                unit_lm
            ), "lexicon free decoding can only be done with a unit language model"

            d = {w: [[w]] for w in token_dict.symbols}
            self.word_dict = create_word_dict(d)
            self.lm = KenLM(lang_model, self.word_dict)

            self.decoder_opts = LexiconFreeDecoderOptions(
                beam_size=beam,
                beam_size_token=(
                    beam_size_token if beam_size_token else len(token_dict)
                ),
                beam_threshold=beam_threshold,
                lm_weight=lm_weight,
                sil_score=sil_weight,
                log_add=False,
                criterion_type=self.criterion_type,
            )

            self.decoder = LexiconFreeDecoder(
                self.decoder_opts, self.lm, self.silence, self.blank, []
            )

    def decode(self, emissions):
        B, T, N = emissions.size()
        hypos = []

        def idx_to_word(idx):
            if self.unit_lm:
                return self.idx_to_wrd[idx]
            else:
                return self.word_dict[idx]

        def make_hypo(result):
            hypo = {"tokens": self.get_tokens(result.tokens), "score": result.score}
            if self.lexicon:
                hypo["words"] = [idx_to_word(x) for x in result.words if x >= 0]
            return hypo

        for b in range(B):
            emissions_ptr = emissions.data_ptr() + 4 * b * emissions.stride(0)
            results = self.decoder.decode(emissions_ptr, T, N)

            nbest_results = results[: self.nbest]
            hypos.append([make_hypo(result) for result in nbest_results])
            self.lm.empty_cache()

        return hypos
