import copy
import unittest
from stationary_ble_gate import assess


class GateTests(unittest.TestCase):
    def setUp(self):
        self.data = {'error': None, 'decode_errors': [],
            'info': {'boot_id': 7, 'firmware': [0,11,4]},
            'final_info': {'boot_id': 7, 'firmware': [0,11,4]},
            'before': {'state':1,'flags':35}, 'after': {'state':1,'flags':35},
            'running': {'streaming':True,'rate_hz':20,'channel_mask':7,'dropped_sample_count':0,'stream_sequence':1},
            'stopped': {'streaming':False,'rate_hz':20,'channel_mask':7,'dropped_sample_count':0,'stream_sequence':1},
            'frames': [{'stream_sequence':1,'sample_sequence':i+1,'timestamp_ms':1000+i*50,
                'flags':11,'channel':c,'activation_epoch_lsb':1,
                'left_raw':((-1)**i*10 if c==2 and i%3==0 else 0),'right_raw':0}
                for i in range(60) for c in (1,2,4)]}

    def test_valid_and_stop_boundary(self):
        self.assertEqual(assess(self.data,7)[0]['samples'],60)
        self.data['frames'].pop()
        self.assertEqual(assess(self.data,7)[0]['samples'],59)

    def test_frame_faults(self):
        for key,value in [('flags',15),('flags',75),('activation_epoch_lsb',2),
                          ('stream_sequence',2),('timestamp_ms',9999),('left_raw',None)]:
            d=copy.deepcopy(self.data);d['frames'][20][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(RuntimeError):assess(d,7)

    def test_missing_duplicate_or_motion(self):
        d=copy.deepcopy(self.data);del d['frames'][30]
        with self.assertRaises(RuntimeError):assess(d,7)
        d=copy.deepcopy(self.data);d['frames'].append(d['frames'][30])
        with self.assertRaises(RuntimeError):assess(d,7)
        d=copy.deepcopy(self.data);d['frames'][30]['left_raw']=10
        with self.assertRaises(RuntimeError):assess(d,7)

    def test_boot_drop_and_decode(self):
        for key,field,value in [('final_info','boot_id',8),('running','dropped_sample_count',1),('after','state',3)]:
            d=copy.deepcopy(self.data);d[key][field]=value
            with self.assertRaises(RuntimeError):assess(d,7)
        self.data['decode_errors']=['bad frame']
        with self.assertRaises(RuntimeError):assess(self.data,7)


if __name__=='__main__':unittest.main()
