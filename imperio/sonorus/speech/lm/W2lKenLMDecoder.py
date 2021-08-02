from .W2lDecoder import W2lDecoder

from flashlight.lib.text.dictionary import create_word_dict, load_words
from flashlight.lib.text.decoder import (
    LexiconDecoderOptions,
    LexiconFreeDecoderOptions,
    KenLM,
    Trie,
    SmearingMode,
    LexiconDecoder,
    LexiconFreeDecoder,
)


class W2lKenLMDecoder(W2lDecoder):
    def __init__(
        self,
        token_dict,
        lexicon,
        lang_model,
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

        if lexicon:
            self.lexicon = load_words(lexicon)
            self.word_dict = create_word_dict(self.lexicon)
            self.unk_word = self.word_dict.get_index("<unk>")

            self.lm = KenLM(lang_model, self.word_dict)
            self.trie = Trie(self.vocab_size, self.silence)

            start_state = self.lm.start(False)
            for i, (word, spellings) in enumerate(self.lexicon.items()):
                word_idx = self.word_dict.get_index(word)
                _, score = self.lm.score(start_state, word_idx)
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

            if self.asg_transitions is None:
                N = 768
                # self.asg_transitions = torch.FloatTensor(N, N).zero_()
                self.asg_transitions = []

            self.decoder = LexiconDecoder(
                self.decoder_opts,
                self.trie,
                self.lm,
                self.silence,
                self.blank,
                self.unk_word,
                self.asg_transitions,
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
        for b in range(B):
            emissions_ptr = emissions.data_ptr() + 4 * b * emissions.stride(0)
            results = self.decoder.decode(emissions_ptr, T, N)

            nbest_results = results[: self.nbest]
            hypos.append(
                [
                    {
                        "tokens": self.get_tokens(result.tokens),
                        "score": result.score,
                        "words": [
                            self.word_dict.get_entry(x) for x in result.words if x >= 0
                        ],
                    }
                    for result in nbest_results
                ]
            )
        return hypos
