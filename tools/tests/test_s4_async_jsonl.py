import json
from pathlib import Path
from threading import Event
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s4_async_jsonl import JsonlWriter


def test_delayed_disk_write_does_not_block_telemetry_admission(tmp_path,monkeypatch):
    entered=Event();release=Event();original=Path.open
    class Delayed:
        def __init__(self,path):self.stream=original(path,'a')
        def write(self,value):entered.set();assert release.wait(2.);return self.stream.write(value)
        def flush(self):self.stream.flush()
        def close(self):self.stream.close()
    monkeypatch.setattr(Path,'open',lambda path,*args,**kwargs:Delayed(path))
    writer=JsonlWriter(tmp_path);first={'frame':1};writer.emit('samples',first)
    assert entered.wait(2.)
    first['frame']=999  # Queued evidence is immutable despite caller updates.
    writer.emit('samples',{'frame':2})
    assert writer.queue.qsize()==1
    release.set();writer.close()
    with original(tmp_path/'samples.jsonl') as f:assert [json.loads(x) for x in f]==[{'frame':1},{'frame':2}]
