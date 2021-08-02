from pathlib import Path

__version__ = "0.2.0"

CACHE_DIR = Path.home() / ".cache/sonorus"
CACHE_DIR.mkdir(parents=True, exist_ok=True)
