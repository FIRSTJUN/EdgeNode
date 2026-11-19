"""Pin executed Python, native implementation source and compiled extension."""
import hashlib
import json
from pathlib import Path


def implementation_files():
    root=Path(__file__).parent
    return tuple(sorted({*root.glob('*.py'),*root.glob('*.cpp'),*root.glob('_geometry_native*.so')}))


def implementation_manifest():
    return {f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in implementation_files()}


def implementation_sha256():
    return hashlib.sha256(json.dumps(implementation_manifest(),sort_keys=True).encode()).hexdigest()
