from pathlib import Path
import pandas as pd
import re

actuator_names_file = Path(__file__).parent / "actuator_names.txt"
ACTUATOR_NAMES = pd.read_csv(actuator_names_file, header=None).iloc[:, 0]

head_actuator_keywords = [
    # "Brow",
    # "Lid",
    # "Eye",
    "Cheek",
    "Lip",
    "Frown",
    # "Smile",
    # "Jaw",
    "Tongue",
    # "Gimbal",
    # "Neck",
]

head_actuator_regex = "({})".format("|".join(head_actuator_keywords))
head_actuators = ACTUATOR_NAMES.str.contains(head_actuator_regex, regex=True, na=False)
HEAD_ACTUATOR_NAMES = ACTUATOR_NAMES.loc[head_actuators]
