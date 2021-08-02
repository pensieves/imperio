import itertools as it
import warnings

import torch

from flashlight.lib.text.dictionary import unpack_replabels
from flashlight.lib.text.decoder import CriterionType
from fairseq.data.data_utils import post_process


class W2lDecoder(object):
    def __init__(
        self,
        token_dict,
        nbest=1,
        criterion="ctc",
        asg_transitions=None,
        max_replabel=None,
    ):
        self.token_dict = token_dict
        self.vocab_size = len(token_dict)
        self.nbest = nbest

        # criterion-specific init
        if criterion == "ctc":
            self.criterion_type = CriterionType.CTC
            self.blank = (
                token_dict.index("<ctc_blank>")
                if "<ctc_blank>" in token_dict.indices
                else token_dict.bos()
            )
            if "<sep>" in token_dict.indices:
                self.silence = token_dict.index("<sep>")
            elif "|" in token_dict.indices:
                self.silence = token_dict.index("|")
            else:
                self.silence = token_dict.eos()
            self.asg_transitions = None

        elif criterion == "asg_loss":
            self.criterion_type = CriterionType.ASG
            self.blank = -1
            self.silence = -1
            self.asg_transitions = asg_transitions
            self.max_replabel = max_replabel
            assert len(self.asg_transitions) == self.vocab_size ** 2

        else:
            raise RuntimeError(f"unknown criterion: {criterion}")

    def get_tokens(self, idxs):
        """Normalize tokens by handling CTC blank, ASG replabels, etc."""
        idxs = (g[0] for g in it.groupby(idxs))
        if self.criterion_type == CriterionType.CTC:
            idxs = filter(lambda x: x != self.blank, idxs)
        elif self.criterion_type == CriterionType.ASG:
            idxs = filter(lambda x: x >= 0, idxs)
            idxs = unpack_replabels(list(idxs), self.token_dict, self.max_replabel)
        return torch.LongTensor(list(idxs))

    def post_process(self, decoded_hypos, process_strategy="letter"):
        r"""For supported set  of process strategy see the supported post process
        symbols in fairseq.data.data_utils.post_process"""

        symbol_idx_to_ignore = (
            self.token_dict.pad_index,
            self.token_dict.bos_index,
            self.token_dict.eos_index,
        )

        batch_sents = []
        for hypos in decoded_hypos:

            sents = []
            for hypo in hypos[: min(len(hypos), self.nbest)]:

                hyp_pieces = self.token_dict.string(
                    hypo["tokens"].int().cpu(),
                    escape_unk=True,
                    unk_string=self.token_dict.unk_index,
                    extra_symbols_to_ignore=symbol_idx_to_ignore,
                )

                if "words" in hypo:
                    sents.append(" ".join(hypo["words"]))
                else:
                    sents.append(post_process(hyp_pieces, process_strategy))

            batch_sents.append(sents)

        return batch_sents
