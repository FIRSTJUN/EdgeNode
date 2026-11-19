import hashlib,json
from pathlib import Path

def manifest():
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path(__file__).parent.glob('*.py'))}

def implementation_sha256():
    return hashlib.sha256(json.dumps(manifest(),sort_keys=True).encode()).hexdigest()
