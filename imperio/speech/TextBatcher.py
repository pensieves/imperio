import re

BATCH_CHUNK = 15  # in terms of tokens
TERMINATING_CHARS = ".?"


class TextBatcher(object):
    def __init__(self, batch_chunk=BATCH_CHUNK, terminating_chars=TERMINATING_CHARS):
        self.batch_chunk = batch_chunk
        self.terminating_regex = "[{}]".format(terminating_chars)
        self.residual_text = ""
        self.processed_text_len = 0

    def reset(self):
        self.residual_text = ""
        self.processed_text_len = 0

    def get_batch(self, text, reset=False):
        cur_text_len = len(text)
        text = text[self.processed_text_len :]
        tokenized = text.split()

        if len(tokenized) >= self.batch_chunk or reset:
            num_batch = len(tokenized) // self.batch_chunk
            if len(tokenized) % self.batch_chunk:
                num_batch += 1

            batch = [
                " ".join(tokenized[i * self.batch_chunk : (i + 1) * self.batch_chunk])
                for i in range(num_batch)
            ]
            batch[0] = self.residual_text + batch[0]
            self.residual_text = self.get_residual_text(batch, tokenized, reset=reset)

            if reset:
                self.reset()
            else:
                self.processed_text_len = cur_text_len

            # print("**")
            # print(batch, self.residual_text)
            # print("**")

            return batch

    def get_residual_text(self, batch, tokenized, reset=False):
        residual = ""

        if len(batch) > 1 and len(tokenized) % self.batch_chunk:
            if reset:
                batch[-2] += " " + batch[-1]
                batch.pop()

            else:
                residual = batch.pop()
                terminating_char_pos = re.search(self.terminating_regex, residual)

                if terminating_char_pos:
                    terminating_char_pos = terminating_char_pos.start() + 1
                    batch[-1] += " " + residual[:terminating_char_pos]
                    residual = residual[terminating_char_pos:].lstrip()

                if residual:
                    residual += " "

        return residual
