import yaml
from pathlib import Path

visemes_config_yaml = Path(__file__).parent / "visemes_config.yaml"


def get_phoneme2viseme(
    config_yaml=visemes_config_yaml, mapping="operator_voice", drop_params=set()
):

    with open(config_yaml, "r") as f:
        viseme_config = yaml.safe_load(f)

    viseme2phoneme = viseme_config["mappings"][mapping]
    viseme_params = viseme_config["params"][mapping]

    phoneme2viseme = dict()
    for viseme, phonemes in viseme2phoneme.items():
        phoneme2viseme.update({p: viseme for p in phonemes})

    # viseme for silence
    sil = "polly_sil" if mapping == "polly_full" else "Sil"

    if drop_params:
        viseme_params = {
            viseme: {
                param: val for param, val in params.items() if param not in drop_params
            }
            for viseme, params in viseme_params.items()
        }

    return phoneme2viseme, sil, viseme_params
