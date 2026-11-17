"""Freeze the Planning source manifest once per interpreter, before driving."""
from functools import lru_cache
import hashlib
from pathlib import Path


@lru_cache(maxsize=1)
def implementation_sha256():
    digest=hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        digest.update(path.name.encode()+b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()
