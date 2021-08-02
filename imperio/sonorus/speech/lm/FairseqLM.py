import gc
from collections import deque, namedtuple

import numpy as np
import torch
from fairseq.utils import apply_to_sample

from flashlight.lib.text.decoder import LM, LMState


FairseqLMState = namedtuple("FairseqLMState", ["prefix", "incremental_state", "probs"])


class FairseqLM(LM):
    def __init__(self, word_dict, model, model_device=torch.device("cpu")):

        LM.__init__(self)

        self.word_dict = word_dict
        self.model = model
        self.model_device = model_device
        self.unk = self.word_dict.unk()

        self.save_incremental = False  # this currently does not work properly
        self.max_cache = 20_000

        self.states = {}
        self.stateq = deque()

    def start(self, start_with_nothing):
        state = LMState()
        prefix = torch.LongTensor([[self.word_dict.eos()]])
        incremental_state = {} if self.save_incremental else None

        with torch.no_grad():
            res = self.model(
                prefix.to(self.model_device), incremental_state=incremental_state
            )
            probs = self.model.get_normalized_probs(res, log_probs=True, sample=None)

        if incremental_state is not None:
            incremental_state = apply_to_sample(lambda x: x.cpu(), incremental_state)

        self.states[state] = FairseqLMState(
            prefix.numpy(), incremental_state, probs[0, -1].cpu().numpy()
        )

        self.stateq.append(state)

        return state

    def score(self, state: LMState, token_index: int, no_cache: bool = False):
        """
        Evaluate language model based on the current lm state and new word
        Parameters:
        -----------
        state: current lm state
        token_index: index of the word
                     (can be lexicon index then you should store inside LM the
                      mapping between indices of lexicon and lm, or lm index of a word)

        Returns:
        --------
        (LMState, float): pair of (new state, score for the current word)
        """
        curr_state = self.states[state]

        def trim_cache(targ_size):
            while len(self.stateq) > targ_size:
                rem_k = self.stateq.popleft()
                rem_st = self.states[rem_k]
                rem_st = FairseqLMState(rem_st.prefix, None, None)
                self.states[rem_k] = rem_st

        if curr_state.probs is None:
            new_incremental_state = (
                curr_state.incremental_state.copy()
                if curr_state.incremental_state is not None
                else None
            )

            with torch.no_grad():
                if new_incremental_state is not None:
                    new_incremental_state = apply_to_sample(
                        lambda x: x.to(self.model_device), new_incremental_state
                    )

                elif self.save_incremental:
                    new_incremental_state = {}

                res = self.model(
                    torch.from_numpy(curr_state.prefix).to(self.model_device),
                    incremental_state=new_incremental_state,
                )

                probs = self.model.get_normalized_probs(
                    res, log_probs=True, sample=None
                )

                if new_incremental_state is not None:
                    new_incremental_state = apply_to_sample(
                        lambda x: x.cpu(), new_incremental_state
                    )

                curr_state = FairseqLMState(
                    curr_state.prefix, new_incremental_state, probs[0, -1].cpu().numpy()
                )

            if not no_cache:
                self.states[state] = curr_state
                self.stateq.append(state)

        score = curr_state.probs[token_index].item()

        trim_cache(self.max_cache)

        outstate = state.child(token_index)
        if outstate not in self.states and not no_cache:
            prefix = np.concatenate(
                [curr_state.prefix, torch.LongTensor([[token_index]])], -1
            )
            incr_state = curr_state.incremental_state

            self.states[outstate] = FairseqLMState(prefix, incr_state, None)

        if token_index == self.unk:
            score = float("-inf")

        return outstate, score

    def finish(self, state: LMState):
        """
        Evaluate eos for language model based on the current lm state

        Returns:
        --------
        (LMState, float): pair of (new state, score for the current word)
        """
        return self.score(state, self.word_dict.eos())

    def empty_cache(self):
        self.states = {}
        self.stateq = deque()
        gc.collect()
