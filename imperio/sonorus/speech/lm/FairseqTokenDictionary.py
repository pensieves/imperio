from fairseq.data import Dictionary


class FairseqTokenDictionary(Dictionary):
    """A mapping from symbols to consecutive integers. 
    A modified class which can take a python dictionary of 
    token vocab with already mapped integer values."""

    def __init__(
        self, bos="<s>", pad="<pad>", eos="</s>", unk="<unk>", indexed_symbols=None
    ):
        special_symbols = (bos, pad, eos, unk)
        self.bos_word, self.pad_word, self.eos_word, self.unk_word = special_symbols
        self.symbols = []
        self.count = []
        self.indices = {}

        if indexed_symbols is None:
            indexed_symbols = special_symbols
        else:
            assert_str = "special symbol {} should be in the provided indexed_symbols"
            for s in special_symbols:
                assert s in indexed_symbols, assert_str.format(s)

            if isinstance(indexed_symbols, dict):
                indexed_symbols = sorted(
                    indexed_symbols.keys(), key=lambda k: indexed_symbols[k]
                )

        for s in indexed_symbols:
            self.add_symbol(s)

        self.bos_index = self.index(bos)
        self.pad_index = self.index(pad)
        self.eos_index = self.index(eos)
        self.unk_index = self.index(unk)
        self.nspecial = len(special_symbols)
