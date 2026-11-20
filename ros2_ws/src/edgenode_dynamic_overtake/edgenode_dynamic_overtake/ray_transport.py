"""Lossless local rear-ray transport; canonical archive keeps inline XYZ.

Scratch frames are immutable, source stamped and hashed. A missing/corrupt
frame fails closed. The full inline ROS evidence is independently recordable.
"""
import hashlib,struct,time,uuid
from pathlib import Path
import numpy as np

ROOT=Path('/tmp/edgenode_d_ray_frames')
HEADER=struct.Struct('<8sQ')
MAGIC=b'EDNDRAY1'

class RayFiles:
    def __init__(self):
        self.directory=ROOT/uuid.uuid4().hex;self.directory.mkdir(parents=True,exist_ok=False);self.files=[]
    def write(self,raw,source_ns,kind):
        payload=HEADER.pack(MAGIC,source_ns)+raw
        final=self.directory/(kind+'-'+str(source_ns)+'.bin');temporary=final.with_suffix('.pending')
        if final.exists():
            if final.read_bytes()!=payload:raise ValueError('D immutable source frame changed')
        else:
            temporary.write_bytes(payload);temporary.replace(final)
        self.files.append((time.monotonic(),final))
        cutoff=time.monotonic()-4.
        while self.files and self.files[0][0]<cutoff:
            _,old=self.files.pop(0);old.unlink(missing_ok=True)
        return dict(path=str(final),sha256=hashlib.sha256(payload).hexdigest(),bytes=len(payload),source_ns=source_ns)

def read(ref,source_ns):
    path=Path(ref['path']).resolve(strict=True)
    if not path.is_relative_to(ROOT.resolve()):raise ValueError('D ray pointer outside owned scratch root')
    data=path.read_bytes()
    if len(data)!=ref['bytes'] or hashlib.sha256(data).hexdigest()!=ref['sha256']:raise ValueError('D ray content integrity failure')
    magic,epoch=HEADER.unpack_from(data)
    if magic!=MAGIC or epoch!=source_ns or ref['source_ns']!=source_ns:raise ValueError('D ray source epoch mismatch')
    return np.frombuffer(data,dtype='<f8',offset=HEADER.size).reshape((-1,3))


def verify_inline(raw,source_ns,reference):
    data=HEADER.pack(MAGIC,source_ns)+raw
    if reference['source_ns']!=source_ns or len(data)!=reference['bytes'] or hashlib.sha256(data).hexdigest()!=reference['sha256']:raise ValueError('D inline archive differs from source ray pointer')
