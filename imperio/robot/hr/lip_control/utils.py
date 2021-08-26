import yaml
import random
from pathlib import Path

visemes_config_yaml = Path(__file__).parent / "visemes_config.yaml"


def get_viseme2phoneme(
    config_yaml=visemes_config_yaml, mapping="operator_voice", drop_params=set()
):

    with open(config_yaml, "r") as f:
        viseme_config = yaml.safe_load(f)

    viseme2phoneme = viseme_config["mappings"][mapping]
    viseme_params = viseme_config["params"][mapping]

    # viseme for silence
    sil = "polly_sil" if mapping == "polly_full" else "Sil"

    if drop_params:
        viseme_params = {
            viseme: {
                param: val for param, val in params.items() if param not in drop_params
            }
            for viseme, params in viseme_params.items()
        }

    return viseme2phoneme, sil, viseme_params


def get_phoneme2viseme(mapping="operator_voice", drop_params=set()):

    viseme2phoneme, sil, viseme_params = get_viseme2phoneme(
        mapping=mapping, drop_params=drop_params,
    )

    phoneme2viseme = dict()
    for viseme, phonemes in viseme2phoneme.items():
        phoneme2viseme.update({p: viseme for p in phonemes})

    # viseme for silence
    sil = "polly_sil" if mapping == "polly_full" else "Sil"

    return phoneme2viseme, sil, viseme_params


def drop_random(phone_visem_list, seed=None, drop_th=0.8):

    random_gen = random.Random(seed)
    phone_visem_list = [i for i in phone_visem_list if random_gen.random() >= drop_th]

    start = 0
    for phone_visem in phone_visem_list:
        phone_visem["start"] = start
        start += phone_visem["duration"]

    return phone_visem_list


def shift_time(phone_visem_list, shift=0):
    if shift:
        phone_visem_list = deepcopy(phone_visem_list)
        for phone_visem in phone_visem_list:
            phone_visem["start"] += shift
    return phone_visem_list
