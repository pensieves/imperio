""" Common Voice Dataset V1 2017-12-12. Code has been adapted for old version v1 from
https://github.com/huggingface/datasets/blob/master/datasets/common_voice/common_voice.py
which is for latest version. Also visit https://huggingface.co/docs/datasets/add_dataset.html for further understanding.
"""

from pathlib import Path
import glob
import pandas as pd

import datasets


_DESCRIPTION = """
Common Voice is Mozilla's initiative to help teach machines how real people speak.
This dataset currently consists of English speech from the v1 dataset available on kaggle.
"""

_HOMEPAGE = "https://www.kaggle.com/mozillaorg/common-voice"

_LICENSE = "https://www.kaggle.com/mozillaorg/common-voice?select=LICENSE.txt"

_CITATION = ""

_LANGUAGES = {
    "en": {
        "Language": "English",
        "Date": "2017-12-12",
        "Size": "13 GB",
        "Version": "en_469h_2017-12-12",
        "Validated_Hr_Total": 252,
        "Overall_Hr_Total": 469,
    }
}


class CommonVoiceConfig(datasets.BuilderConfig):
    """BuilderConfig for CommonVoice."""

    def __init__(self, name="en", sub_version="1.0.0", **kwargs):
        """
        Args:
          data_dir: `string`, the path to the folder containing the downloaded files.
          **kwargs: keyword arguments forwarded to super.
        """
        self.sub_version = sub_version
        self.language = kwargs.pop("language", None)
        self.date_of_snapshot = kwargs.pop("date", None)
        self.size = kwargs.pop("size", None)
        self.validated_hr_total = kwargs.pop("val_hrs", None)
        self.total_hr_total = kwargs.pop("total_hrs", None)
        description = f"Common Voice speech to text dataset in {self.language} version {self.sub_version} of {self.date_of_snapshot}. The dataset comprises {self.validated_hr_total} of validated transcribed speech data. The dataset has a size of {self.size}"
        super(CommonVoiceConfig, self).__init__(
            name=name,
            version=datasets.Version("1.0.0", ""),
            description=description,
            **kwargs,
        )


class CommonVoice(datasets.GeneratorBasedBuilder):

    BUILDER_CONFIGS = [
        CommonVoiceConfig(
            name=lang_id,
            language=_LANGUAGES[lang_id].get("Language"),
            sub_version=_LANGUAGES[lang_id].get("Version"),
            date=_LANGUAGES[lang_id].get("Date"),
            size=_LANGUAGES[lang_id].get("Size"),
            val_hrs=_LANGUAGES[lang_id].get("Validated_Hr_Total"),
            total_hrs=_LANGUAGES[lang_id].get("Overall_Hr_Total"),
        )
        for lang_id in _LANGUAGES.keys()
    ]

    def _info(self):
        features = datasets.Features(
            {
                "filename": datasets.Value("string"),
                "text": datasets.Value("string"),
                "up_votes": datasets.Value("int64"),
                "down_votes": datasets.Value("int64"),
                "age": datasets.Value("string"),
                "gender": datasets.Value("string"),
                "accent": datasets.Value("string"),
                "duration": datasets.Value("float64"),
            }
        )

        return datasets.DatasetInfo(
            description=_DESCRIPTION,
            features=features,
            supervised_keys=None,
            homepage=_HOMEPAGE,
            license=_LICENSE,
            citation=_CITATION,
        )

    def _split_generators(self, dl_manager):
        """Returns SplitGenerators."""
        data_dir = Path(self.config.data_dir).resolve()
        splits = [Path(s).stem[3:] for s in glob.glob(f"{data_dir}/*.csv")]

        return [
            datasets.SplitGenerator(
                name=split.replace("-", "."),
                gen_kwargs={
                    "info_path": data_dir / f"cv-{split}.csv",
                    "audio_dir": data_dir / f"cv-{split}",
                },
            )
            for split in splits
        ]

    def _generate_examples(self, info_path, audio_dir):
        """ Yields examples. """

        data_fields = list(self._info().features.keys())
        df = pd.read_csv(info_path, sep=",")

        # set duration as -1 if not present i.e nan
        df.loc[df.duration.isna(), "duration"] = -1
        cols = list(df.columns)

        assert (
            cols == data_fields
        ), f"The file should have {data_fields} as column names, but has {cols}"

        for id_, field_values in enumerate(df.itertuples(index=False)):

            field_values = pd.Series(field_values._asdict())

            # set absolute path for mp3 audio file
            field_values.filename = str(audio_dir / field_values.filename)

            # if data is incomplete i.e. has nan, fill with empty values
            field_values[field_values.isna()] = ""

            yield id_, {key: value for key, value in zip(data_fields, field_values)}
