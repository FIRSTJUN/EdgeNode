"""Freeze the Planning source manifest once per interpreter, before driving."""
from functools import lru_cache
import hashlib
from pathlib import Path


def implementation_files():
    root=Path(__file__).parent
    return tuple(sorted({*root.glob('*.py'),*root.glob('*.cpp'),*root.glob('_geometry_native*.so')}))


@lru_cache(maxsize=1)
def implementation_sha256():
    digest=hashlib.sha256()
    for path in implementation_files():
        digest.update(path.name.encode()+b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()
