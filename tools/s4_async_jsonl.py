"""Bounded evidence writer; disk latency cannot stall ROS supervision."""
import json
from queue import Queue,Full
from threading import Thread


class JsonlWriter:
    def __init__(self,directory):
        self.directory=directory;self.queue=Queue(maxsize=256);self.error=None
        self.thread=Thread(target=self._run,name='s4-evidence-writer',daemon=True);self.thread.start()

    def _run(self):
        streams={}
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                name,payload=item
                if name not in streams:streams[name]=(self.directory/(name+'.jsonl')).open('a')
                streams[name].write(payload);streams[name].flush()
        except BaseException as error:self.error=error
        finally:
            for stream in streams.values():stream.close()

    def emit(self,name,row):
        if self.error:raise RuntimeError('evidence writer failed: '+str(self.error))
        try:self.queue.put_nowait((name,json.dumps(row)+'\n'))
        except Full:raise RuntimeError('evidence writer backlog exhausted; no records discarded')

    def close(self):
        if self.error:raise RuntimeError('evidence writer failed: '+str(self.error))
        self.queue.put(None,timeout=5.);self.thread.join(timeout=10.)
        if self.thread.is_alive():raise RuntimeError('evidence writer flush not confirmed')
        if self.error:raise RuntimeError('evidence writer failed: '+str(self.error))
