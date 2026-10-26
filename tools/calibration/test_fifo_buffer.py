import unittest
from fifo_serial_buffer import BufferedSerial


class Port:
    def __init__(self,data,chunk):
        self.data=bytearray(data)
        self.chunk=chunk
        self.is_open=True
    @property
    def in_waiting(self):
        return min(self.chunk,len(self.data))
    def read(self,n):
        output=bytes(self.data[:n]);del self.data[:n]
        return output
    def write(self,data):
        return len(data)
    def close(self):
        self.is_open=False


class BufferTests(unittest.TestCase):
    def test_all_chunk_boundaries_preserve_exact_records(self):
        data=b'QUERY_REPLY\r\nS,0,1,-2,3,4,5,6,7\nD,0,1,2,3\r\nEND,20000\n'
        expected=data.splitlines(keepends=True)
        for size in range(1,len(data)+1):
            reader=BufferedSerial(Port(data,size))
            self.assertEqual([reader.readline() for _ in expected],expected)
            self.assertEqual(reader.readline(),b'')

    def test_large_batch_needs_fewer_reads_than_byte_reader(self):
        reader=BufferedSerial(Port(b'S,0\n'*1000,65536))
        for _ in range(1000):
            self.assertEqual(reader.readline(),b'S,0\n')
        self.assertLess(reader.read_calls,10)

    def test_truncated_record_is_returned_without_invented_newline(self):
        reader=BufferedSerial(Port(b'S,0,partial',20))
        self.assertEqual(reader.readline(),b'S,0,partial')


if __name__=='__main__':
    unittest.main()
