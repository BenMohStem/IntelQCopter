"""Synthetic negative controls for the PC checker; no device access."""
import copy
import importlib.util
from pathlib import Path
import struct
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('comparison',ROOT/'tools/validation/compare_imu_acquisition.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class AnalysisChecks(unittest.TestCase):
    def setUp(self):
        self.rows=[[i,i*1000,i*1000+100,i*1000+200,i*1000+651,0,1,2,4096,0,4,5,6] for i in range(100)]
        checksum=2166136261
        for row in self.rows:
            for value in struct.pack('>7h',*row[6:]):checksum=((checksum^value)*16777619)&0xffffffff
        self.meta={'mode':'d','samples':100,'elapsed_us':4000000,'injected_pauses':0,'tail_bytes':0,'checksum':checksum}

    def test_known_durations_and_units(self):
        result=module.summarize(self.rows,self.meta)
        self.assertEqual(result['data_read_us']['median'],451)
        self.assertEqual(result['successful_service_us']['median'],651)
        self.assertEqual(result['gyro_mean_raw'],[4,5,6])

    def test_corrupt_payload(self):
        self.rows[10][6]+=1
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_missing_sequence(self):
        del self.rows[10]
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_timestamp_order(self):
        self.rows[10][4]=self.rows[10][3]-1
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_wrong_window(self):
        self.meta['elapsed_us']=3999999
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_fake_fifo_in_direct(self):
        self.rows[10][5]=14
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_wrong_pause_condition(self):
        self.meta['mode']='D'
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_valid_fifo_and_impossible_fifo(self):
        self.meta['mode']='f'
        for row in self.rows:row[5]=42
        result=module.summarize(self.rows,self.meta)
        self.assertEqual(result['fifo_oldest_extra_queue_age_proxy_us'],2000)
        self.rows[10][5]=1024
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)


if __name__=='__main__':unittest.main(verbosity=2)
