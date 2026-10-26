"""Buffered serial line transport; preserves bytes and never repairs damaged records."""
import time


class BufferedSerial:
    def __init__(self,port):
        self.port=port
        self.pending=bytearray()
        self.read_calls=0

    @property
    def is_open(self):
        return self.port.is_open

    def write(self,data):
        return self.port.write(data)

    def close(self):
        return self.port.close()

    def readline(self):
        deadline=time.monotonic()+1
        while True:
            end=self.pending.find(b'\n')
            if end>=0:
                result=bytes(self.pending[:end+1])
                del self.pending[:end+1]
                return result
            if len(self.pending)>4096:
                raise ValueError('Oversized serial record; no bytes repaired or discarded')
            if time.monotonic()>=deadline:
                result=bytes(self.pending)
                self.pending.clear()
                return result
            chunk=self.port.read(max(1,min(self.port.in_waiting,65536)))
            self.read_calls+=1
            if not chunk:
                result=bytes(self.pending)
                self.pending.clear()
                return result
            self.pending.extend(chunk)
