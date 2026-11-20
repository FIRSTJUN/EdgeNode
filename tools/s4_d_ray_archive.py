#!/usr/bin/env python3
"""Preserve immutable D ray files during a trial; no ROS/Actor/control input.

This observer never grants planning permission. It copies complete atomic
source files before the bounded transport ring expires, preserving their
original header and bytes when large DDS archive messages are dropped.
"""
import hashlib,json,os,shutil,signal,struct,time
from pathlib import Path

class RayArchive:
    def __init__(self,directory):
        self.directory=directory;directory.mkdir(exist_ok=False)
        self.seen=set();self.errors=[];self.oldest_mtime=time.time()-.6
    def capture(self):
        for source in Path('/tmp/edgenode_d_ray_frames').glob('*/*.bin'):
            key=(source.parent.name,source.name)
            if key in self.seen:continue
            try:
                if source.stat().st_mtime<self.oldest_mtime:self.seen.add(key);continue
            except FileNotFoundError:continue
            destination=self.directory/source.parent.name/source.name
            destination.parent.mkdir(exist_ok=True)
            try:
                try:os.link(source,destination)
                except OSError:shutil.copyfile(source,destination)
            except FileNotFoundError:continue
            self.seen.add(key)
    def finalize(self):
        self.capture();rows=[]
        for filename in sorted(self.directory.glob('*/*.bin')):
            data=filename.read_bytes();magic,source_ns=struct.unpack_from('<8sQ',data)
            assert magic==b'EDNDRAY1' and (len(data)-16)%24==0
            rows.append(dict(path=str(filename.relative_to(self.directory)),source_ns=source_ns,sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)))
        (self.directory/'manifest.json').write_text(json.dumps(dict(scope='EXACT_ATOMIC_TRANSPORT_BYTES_PRESERVED_BY_READ_ONLY_TRIAL_OBSERVER',files=rows),indent=2))
        return len(rows)

def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path);a=parser.parse_args()
    observer=RayArchive(a.output);running=True
    def stop(*args):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        while running:observer.capture();time.sleep(.1)
    finally:print('Archived immutable ray sources:',observer.finalize(),flush=True)

if __name__=='__main__':main()
