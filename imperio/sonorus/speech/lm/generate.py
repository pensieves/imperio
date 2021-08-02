import argparse
import gzip
import io
from pathlib import Path
import subprocess
from collections import Counter
from tqdm import tqdm
import logging

LOGGING_LEVEL = logging.INFO
LOGGING_FORMAT = "%(asctime)s %(message)s"
LOGGING_DATEFMT = "%m/%d/%Y %I:%M:%S %p"

logging.basicConfig(level=LOGGING_LEVEL, format=LOGGING_FORMAT, datefmt=LOGGING_DATEFMT)


def generate_data_and_vocab_file(
    inp_corpus_path, top_k, output_dir, case="upper", spell=True, spell_end_token="|"
):
    """ Convert to specified case, count word occurrences and save top-k words to a file """

    logging.info(
        f"Converting to the specified {case} case and counting word occurrences ..."
    )

    output_dir = Path(output_dir)
    data_file = output_dir / "data.txt.gz"
    vocab_file = output_dir / f"vocab-{top_k}.txt"

    word_counter = generate_data_file_and_word_counter(inp_corpus_path, data_file, case)
    vocab, top_counter = generate_vocab_file(
        word_counter, top_k, vocab_file, spell, spell_end_token
    )
    log_counter_stats(word_counter, top_k, top_counter)

    return data_file, vocab


def generate_data_file_and_word_counter(inp_corpus_path, data_file, case="upper"):

    word_counter = Counter()

    with io.TextIOWrapper(
        io.BufferedWriter(gzip.open(data_file, "w+")), encoding="utf-8"
    ) as file_out:

        # Open the input file either from input.txt or input.txt.gz
        if Path(inp_corpus_path).suffix == ".gz":
            file_in = io.TextIOWrapper(
                io.BufferedReader(gzip.open(inp_corpus_path)), encoding="utf-8"
            )

        else:
            file_in = open(inp_corpus_path, encoding="utf-8")

        for line in tqdm(file_in):
            # lower or upper as specified by case
            line = getattr(line, case)()
            word_counter.update(line.split())
            file_out.write(line)

        file_in.close()

    return word_counter


def generate_vocab_file(
    word_counter, top_k, vocab_file, spell=True, spell_end_token="|"
):

    # Save top-k words
    logging.info(f"Saving top {top_k} words ...")

    top_counter = word_counter.most_common(top_k)
    vocab = [word for word, count in top_counter]

    with open(vocab_file, "w+") as file:
        for word in vocab:
            if spell:
                word = f"{word}\t{' '.join(word)} {spell_end_token}"
                file.write(word + "\n")

    return vocab, top_counter


def log_counter_stats(word_counter, top_k, top_counter):

    logging.info("Calculating word statistics ...")

    total_words = sum(word_counter.values())

    logging.info(f"  Your text file has {total_words} words in total")
    logging.info(f"  It has {len(word_counter)} unique words")

    top_words_sum = sum(count for word, count in top_counter)
    word_fraction = (top_words_sum / total_words) * 100

    logging.info(
        "  Your top-{} words are {:.4f} percent of all words".format(
            top_k, word_fraction
        )
    )

    logging.info(
        '  Your most common word "{}" occurred {} times'.format(*top_counter[0])
    )

    last_word, last_count = top_counter[-1]

    logging.info(
        '  The least common word in your top-k is "{}" with {} times'.format(
            last_word, last_count
        )
    )

    for i, (w, c) in enumerate(reversed(top_counter)):
        if c > last_count:
            logging.info(
                '  The first word with {} occurrences is "{}" at place {}'.format(
                    c, w, len(top_counter) - 1 - i
                )
            )
            break


def build_arpa(
    kenlm_bin_dir,
    data_file,
    lm_path,
    ngram=4,
    max_memory="85%",
    prune="0",
    discount_fallback=False,
):

    logging.info("Creating ARPA file ...")

    subargs = [
        Path(kenlm_bin_dir) / "lmplz",
        "--order",
        str(ngram),
        "--temp_prefix",
        Path(lm_path).parent,
        "--memory",
        max_memory,
        "--text",
        data_file,
        "--arpa",
        lm_path,
        "--prune",
        *prune.split("|"),
    ]

    if discount_fallback:
        subargs += ["--discount_fallback"]

    subprocess.check_call(subargs)


def filter_arpa_for_vocab(kenlm_bin_dir, lm_path, vocab=[]):

    lm_path = Path(lm_path)
    # Filter LM using vocabulary of top-k words
    if vocab:
        logging.info("Filtering ARPA file using vocabulary of top-k words ...")

        filtered_path = lm_path.parent / (lm_path.stem + "_filtered" + lm_path.suffix)

        subprocess.run(
            [
                Path(kenlm_bin_dir) / "filter",
                "single",
                "model:{}".format(lm_path),
                filtered_path,
            ],
            input="\n".join(vocab).encode("utf-8"),
            check=True,
        )
        return filtered_path

    logging.info("No vocab list passed as argument. Filtering not performed.")
    return lm_path


def build_binary(kenlm_bin_dir, lm_path, binary_type="trie", a_bits=255, q_bits=8):

    # Quantize and produce trie binary.
    logging.info("Building binary file ...")

    filtered_idx = lm_path.stem.rfind("_filtered")
    stem = lm_path.stem[:filtered_idx] if filtered_idx > 0 else lm_path.stem
    binary_path = lm_path.parent / (stem + ".binary")

    subprocess.check_call(
        [
            Path(kenlm_bin_dir) / "build_binary",
            "-a",
            str(a_bits),
            "-q",
            str(q_bits),
            "-v",
            binary_type,
            lm_path,
            binary_path,
        ]
    )


def build_lm(
    kenlm_bin_dir,
    data_file,
    lm_path,
    ngram=4,
    max_memory="85%",
    prune="0",
    discount_fallback=False,
    vocab=[],
    binary_type="trie",
    a_bits=255,
    q_bits=8,
):

    build_arpa(
        kenlm_bin_dir, data_file, lm_path, ngram, max_memory, prune, discount_fallback
    )

    filtered_path = filter_arpa_for_vocab(kenlm_bin_dir, lm_path, vocab)

    if binary_type is not None:
        build_binary(kenlm_bin_dir, filtered_path, binary_type, a_bits, q_bits)


def generate_lm(
    inp_corpus_path,
    kenlm_bin_dir,
    top_k,
    output_dir,
    case="upper",
    spell=True,
    spell_end_token="|",
    ngram=4,
    max_memory="85%",
    prune="0",
    discount_fallback=True,
    binary_type="trie",
    a_bits=255,
    q_bits=8,
):

    output_dir = Path(output_dir)
    data_file, vocab = generate_data_and_vocab_file(
        inp_corpus_path, top_k, output_dir, case, spell, spell_end_token
    )

    build_lm(
        kenlm_bin_dir,
        data_file,
        output_dir / "lm.arpa",
        ngram,
        max_memory,
        prune,
        discount_fallback,
        vocab,
        binary_type,
        a_bits,
        q_bits,
    )

    # Delete intermediate files
    Path(data_file).unlink(missing_ok=True)
    (output_dir / "lm.arpa").unlink(missing_ok=True)

    if binary_type is not None:
        (output_dir / "lm_filtered.arpa").unlink(missing_ok=True)
    else:
        (output_dir / "lm_filtered.arpa").rename(output_dir / "lm.arpa")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Generate kenlm model and top-k vocab")

    parser.add_argument(
        "--input_txt",
        help="Path to a file.txt or file.txt.gz with sample sentences",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--case",
        choices=["upper", "lower"],
        default="upper",
        help="case to which the texts and vocab/lexicon will be converted and used",
    )

    parser.add_argument(
        "--spell",
        help="Specify to include spelling in vocab/lexicon file",
        action="store_true",
    )

    parser.add_argument(
        "--spell_end_token",
        default="|",
        help="Token to be used to mark end of the spelling in vocab/lexicon file",
    )

    parser.add_argument(
        "--output_dir", help="Directory path for the output", type=str, required=True
    )

    parser.add_argument(
        "--top_k",
        help="Use top_k most frequent words for the vocab.txt file. These will be used to filter the ARPA file.",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--kenlm_bins",
        help="File path to the KENLM binaries lmplz, filter and build_binary",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--arpa_order",
        help="Order of k-grams in ARPA-file generation",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--max_arpa_memory",
        default="85%",
        help="Maximum allowed memory usage for ARPA-file generation",
    )

    # Specify count threshold for pruning each order. The numbers must be
    # non-decreasing and the last number will be extended to any higher order.
    # For example, --arpa_prune 0 disables pruning (the default) while
    # --arpa_prune "0|0|1" prunes singletons for orders three and higher.
    parser.add_argument(
        "--arpa_prune",
        default="0",
        help="ARPA pruning parameters. Separate values with '|'",
    )

    # specify --binary_type (e.g. trie) to convert LM from arpa to .binary file
    parser.add_argument(
        "--binary_type", help="Build binary data structure type",
    )

    parser.add_argument(
        "--binary_a_bits",
        help="Build binary quantization value a in bits",
        type=int,
        default=255,
    )

    parser.add_argument(
        "--binary_q_bits",
        help="Build binary quantization value q in bits",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--discount_fallback",
        help="To try when such message is returned by kenlm: 'Could not calculate Kneser-Ney discounts [...] rerun with --discount_fallback'",
        action="store_true",
    )

    args = parser.parse_args()

    generate_lm(
        args.input_txt,
        args.kenlm_bins,
        args.top_k,
        args.output_dir,
        case=args.case,
        spell=args.spell,
        spell_end_token=args.spell_end_token,
        ngram=args.arpa_order,
        max_memory=args.max_arpa_memory,
        prune=args.arpa_prune,
        discount_fallback=args.discount_fallback,
        binary_type=args.binary_type,
        a_bits=args.binary_a_bits,
        q_bits=args.binary_q_bits,
    )
