class TextBatchProcessor(object):
    def __init__(self, lang="en-US"):
        r"""language provided should be a BCP-47 language tag."""
        self._lang = lang

    def process(self, batch, **kwargs):
        for text in batch:
            print('Recognised "{}" in "{}"\n'.format(text, self._lang))
