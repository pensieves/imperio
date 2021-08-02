import torch
from flashlight.lib.sequence.criterion import CpuViterbiPath, get_data_ptr_as_bytes

from .W2lDecoder import W2lDecoder


class W2lViterbiDecoder(W2lDecoder):
    def __init__(
        self,
        token_dict,
        nbest=1,
        criterion="ctc",
        asg_transitions=None,
        max_replabel=None,
    ):

        super().__init__(token_dict, nbest, criterion, asg_transitions, max_replabel)

    def decode(self, emissions):

        B, T, N = emissions.size()
        hypos = []

        if self.asg_transitions is None:
            transitions = torch.FloatTensor(N, N).zero_()
        else:
            transitions = torch.FloatTensor(self.asg_transitions).view(N, N)

        viterbi_path = torch.IntTensor(B, T)
        workspace = torch.ByteTensor(CpuViterbiPath.get_workspace_size(B, T, N))

        CpuViterbiPath.compute(
            B,
            T,
            N,
            get_data_ptr_as_bytes(emissions),
            get_data_ptr_as_bytes(transitions),
            get_data_ptr_as_bytes(viterbi_path),
            get_data_ptr_as_bytes(workspace),
        )

        return [
            [{"tokens": self.get_tokens(viterbi_path[b].tolist()), "score": 0}]
            for b in range(B)
        ]
