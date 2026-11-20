#!/usr/bin/env python3
"""Losslessly compress closed, FAILED D prototype bags, with verified restore.

Only SQLite's byte representation changes. Metadata, summaries, raw messages,
source snapshots and logs remain. A full gzip round trip SHA256 match is
required before exchanging the uncompressed representation for the archive.
Completed v61/A/B/C packages are outside the permitted root.
"""
import argparse,gzip,hashlib,json,os,shutil,time
from pathlib import Path

ROOT=Path('/workspace/local_data/s4_d_dynamic_overtake_20261119')
def digest(stream):
    h=hashlib.sha256();size=0
    while block:=stream.read(4*1024*1024):h.update(block);size+=len(block)
    return h.hexdigest(),size
def file_digest(filename):
    with filename.open('rb') as stream:return digest(stream)
def compress(trial):
    trial=trial.resolve();assert trial.parent==ROOT and trial.is_dir()
    summary=json.loads((trial/'summary.json').read_text());assert not summary.get('success') and summary.get('restored')
    assert summary.get('rosbag_exit_code')==0 and summary.get('rosbag_metadata_present')
    bag=trial/'bag';assert (bag/'metadata.yaml').exists()
    receipt=trial/'lossless-bag-preservation.json';assert not receipt.exists()
    files=sorted(bag.glob('*.db3'));assert files
    # No writer may have any source SQLite file open.
    targets={str(p.resolve()) for p in files}
    for proc in Path('/proc').glob('[0-9]*/fd'):
        try:
            for descriptor in proc.iterdir():
                try:target=os.readlink(descriptor)
                except (FileNotFoundError,PermissionError):continue
                assert target not in targets,('bag still open',str(descriptor),target)
        except (FileNotFoundError,PermissionError):continue
    records=[]
    for original in files:
        archived=original.with_suffix('.db3.gz');pending=archived.with_suffix('.gz.pending');assert not archived.exists() and not pending.exists()
        before,size=file_digest(original)
        with original.open('rb') as source,gzip.open(pending,'wb',compresslevel=1) as destination:shutil.copyfileobj(source,destination,4*1024*1024)
        with gzip.open(pending,'rb') as stream:after,decoded_size=digest(stream)
        assert (after,decoded_size)==(before,size)
        archived_sha,archived_size=file_digest(pending);pending.rename(archived)
        record=dict(original=str(original.relative_to(trial)),original_sha256=before,original_bytes=size,archive=str(archived.relative_to(trial)),archive_sha256=archived_sha,archive_bytes=archived_size,full_round_trip_verified=True)
        records.append(record)
        # Save the proof before releasing duplicate byte-for-byte data.
        receipt.write_text(json.dumps(dict(scope='LOSSLESS_REPRESENTATION_OF_CLOSED_FAILED_D_BAGS_NO_MESSAGE_DATA_REMOVED',files=records,restore_command=f'python3 /workspace/tools/s4_closed_bag_preservation.py restore {trial} --output /workspace/local_data/RESTORE_TO_A_NEW_DIRECTORY',complete=False),indent=2))
        original.unlink()
    data=json.loads(receipt.read_text());data.update(complete=True,completed_utc_ns=time.time_ns());receipt.write_text(json.dumps(data,indent=2))
    return data
def restore(trial,output):
    assert not output.exists();receipt=json.loads((trial/'lossless-bag-preservation.json').read_text());assert receipt['complete']
    output.mkdir(parents=True);shutil.copyfile(trial/'bag/metadata.yaml',output/'metadata.yaml')
    for row in receipt['files']:
        archive=trial/row['archive'];assert file_digest(archive)==(row['archive_sha256'],row['archive_bytes'])
        destination=output/Path(row['original']).name
        with gzip.open(archive,'rb') as source,destination.open('xb') as stream:shutil.copyfileobj(source,stream,4*1024*1024)
        assert file_digest(destination)==(row['original_sha256'],row['original_bytes'])
    return dict(restored=str(output),byte_identical_to_original=True)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('compress','restore'));p.add_argument('trial',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    if a.action=='restore':assert a.output is not None
    print(json.dumps(compress(a.trial) if a.action=='compress' else restore(a.trial,a.output),indent=2))
if __name__=='__main__':main()
