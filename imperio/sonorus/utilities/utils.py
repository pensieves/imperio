from pathlib import Path
import yaml
import uuid
import wget
import tarfile, zipfile

from .. import CACHE_DIR


def to_yaml(data, file, safe_dump=True, default_flow_style=False):
    dumper = yaml.safe_dump if safe_dump else yaml.dump
    with open(file, "w") as f:
        dumper(data, f, default_flow_style=default_flow_style)


def from_yaml(file, safe_load=True):
    data = {}
    if Path(file).exists():
        loader = yaml.safe_load if safe_load else yaml.load
        with open(file, "r") as f:
            data = loader(f)
    return data


def create_random_dir(work_dir=CACHE_DIR, prefix="sonorus"):
    random_dir = Path(work_dir) / f"{prefix}_{uuid.uuid4().hex}"
    random_dir.mkdir(parents=True, exist_ok=True)
    return random_dir


def download(url, download_dir, retry=5):

    print(f"Downloading model from {url} ...")
    for i in range(retry):

        try:
            print(f"Attempting download {i+1}/{retry} times ...")
            filename = wget.download(url, out=str(download_dir))
            print("Download completed")
            break

        except Exception as e:
            if i < retry - 1:
                print(f"Exception occured: {e}")

    return filename


def unpack_archive(filename, unpack_dir=None):

    unpacked = list()
    archive_opener = None

    if tarfile.is_tarfile(filename):
        archive_opener = tarfile.open
        names_method = "getnames"

    elif zipfile.is_zipfile(filename):
        archive_opener = zipfile.ZipFile
        names_method = "namelist"

    if archive_opener:
        unpack_dir = Path(unpack_dir) if unpack_dir else Path(filename).parent

        with archive_opener(filename) as f:
            f.extractall(path=unpack_dir)

            unpacked = [
                (unpack_dir / fn).resolve()
                for fn in getattr(f, names_method)()
                if str(Path(fn).parent) == "."
                # i.e only top level directory and filenames
            ]

    return unpacked[0] if len(unpacked) == 1 else unpacked
