import Levenshtein
import numpy as np


overlap = 5
s1 = "Alice and Bob, the stars of so many thought experiments, are cooking dinner when mishaps in"
s2 = "uking dinner when mishaps ensue Alice accidentally drops a plate; the sound startles Bob, who burns himself on the stove and cries out."
# s1 = "Alice and Bob, the"
# s2 = "Bob, the stars of so many thought experiments,"


from STT import STT


stt = STT()
stt._overlapped_append(s1, s2)


stt._model.device


import nltk
from functools import lru_cache
from itertools import product as iterprod

try:
    arpabet = nltk.corpus.cmudict.dict()
except LookupError:
    nltk.download('cmudict')
    arpabet = nltk.corpus.cmudict.dict()

@lru_cache()
def wordbreak(s):
    s = s.lower()
    if s in arpabet:
        return arpabet[s]
    middle = len(s)/2
    partition = sorted(list(range(len(s))), key=lambda x: (x-middle)**2-x)
    for i in partition:
        pre, suf = (s[:i], s[i:])
        if pre in arpabet and wordbreak(suf) is not None:
            return [x+y for x,y in iterprod(arpabet[pre], wordbreak(suf))]
    return None


wordbreak("Rizvi")



