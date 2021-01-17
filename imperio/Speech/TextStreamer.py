import re

STREAM_CHUNK = 15
TERMINATING_CHARS = ".?"

class TextStreamer(object):

    def __init__(self, stream_chunk=STREAM_CHUNK, terminating_chars=TERMINATING_CHARS):
        self.stream_chunk = stream_chunk
        self.terminating_regex = "[{}]".format(terminating_chars)
        self.residual_text = ""
        self.processed_text_len = 0

    def reset(self):
        self.residual_text = ""
        self.processed_text_len = 0

    def text_to_stream(self, text, reset=False):
        cur_text_len = len(text)
        text = text[self.processed_text_len:]
        tokenized = text.split()

        if len(tokenized) >= self.stream_chunk or reset:
            num_chunks = len(tokenized)//self.stream_chunk
            if len(tokenized)%self.stream_chunk:
                num_chunks += 1

            stream = [" ".join(tokenized[i*self.stream_chunk:(i+1)*self.stream_chunk]) 
                        for i in range(num_chunks)]
            stream[0] = self.residual_text + stream[0]
            self.residual_text = self.get_residual_text(stream, tokenized, reset=reset)

            if reset:
                self.reset()
            else:
                self.processed_text_len = cur_text_len

            # print("**")
            # print(stream, self.residual_text)
            # print("**")

            return stream

    def get_residual_text(self, stream, tokenized, reset=False):
        residual = ""

        if len(stream) > 1 and len(tokenized)%self.stream_chunk:
            if reset:
                stream[-2] += (" " + stream[-1])
                stream.pop()

            else:
                residual = stream.pop()
                terminating_char_pos = re.search(self.terminating_regex, residual)

                if terminating_char_pos:
                    terminating_char_pos = terminating_char_pos.start()+1
                    stream[-1] += (" " + residual[:terminating_char_pos])
                    residual = residual[terminating_char_pos:].lstrip()

                if residual:
                    residual += " "                

        return residual