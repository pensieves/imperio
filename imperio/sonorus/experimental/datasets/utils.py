import re
import librosa
from transformers import Wav2Vec2Processor

CHARS_IGNORE_REGEX = '[\,\?\.\!\-\;\:"]'
VOCAB = "vocab"
PAD = "<pad>"
UNK = "<unk>"
BOS = "<s>"
EOS = "</s>"


def get_dataset_col_names(dataset):
    col_names = dataset.column_names
    if not isinstance(col_names, list):
        # Assumptions is all dataset have same column names
        col_names = next(iter(col_names.values()))
    return col_names


def col_filter(batch, col=None, val=None):
    if col is not None and val is not None:
        return batch[col] == val
    return True


def create_speech_array_dataset(
    batch, file_col="filename", sampling_rate=16000, text_col="text"
):

    speech_array, sampling_rate = librosa.load(batch[file_col], sr=sampling_rate)

    batch.update(
        dict(
            speech=speech_array,
            sampling_rate=sampling_rate,
            target_text=batch[text_col],
        )
    )

    return batch


def remove_chars(batch, text_col="text", chars_ignore_regex=CHARS_IGNORE_REGEX):

    batch[text_col] = re.sub(CHARS_IGNORE_REGEX, "", batch[text_col]).upper()
    return batch


def create_vocab(dataset, text_col="text", num_proc=None):
    def extract_vocab(batch, text_col="text"):
        r"""make '|' as part of vocab char set and preferably 
        replace ' ' with '|' for a visible separator.'"""
        return {VOCAB: list(set("|".join(batch[text_col])))}

    vocab_dataset = dataset.map(
        extract_vocab,
        fn_kwargs=dict(text_col=text_col),
        batched=True,
        keep_in_memory=True,
        remove_columns=get_dataset_col_names(dataset),
        num_proc=num_proc,
    )

    vocab_set = set()
    for split in vocab_dataset.keys():
        vocab_set = set.union(vocab_set, vocab_dataset[split][VOCAB])
    # '|' will be used as separator instead of ' '
    vocab_set.remove(" ")

    vocab = {PAD: 0, UNK: 1, BOS: 2, EOS: 3}
    cur_count = len(vocab)
    vocab.update({v: k + cur_count for k, v in enumerate(vocab_set)})

    return vocab


def extract_Wav2Vec2_feats_and_labels(
    batch, processor=Wav2Vec2Processor, speech_col="speech", text_col="target_text"
):

    batch["input_values"] = processor(
        batch[speech_col], sampling_rate=processor.feature_extractor.sampling_rate
    ).input_values

    with processor.as_target_processor():
        batch["labels"] = processor(batch[text_col]).input_ids

    return batch
