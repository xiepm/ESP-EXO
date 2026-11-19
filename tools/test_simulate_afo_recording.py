"""Offline parser and interpolation regression; no device access."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import simulate_afo_recording as sim


def row(t):
    return dict(timestampMs=t, firmware="0.11.21", strategy="ADAPTIVE_GAUSSIAN",
        leftLevelPercent=100, rightLevelPercent=100, maximumNm=15,
        activationEpochLow16=2, webSession=1, streamSequence=1,
        systemState=3, outputAllowed=True, leftPositionDeg=t/10,
        rightPositionDeg=-t/10, leftVelocityDegS=100, rightVelocityDegS=-100)


class RecordingTests(unittest.TestCase):
    def test_gaps_are_not_interpolated(self):
        rows=[row(t) for t in [0,100,200,600,700]]
        t,data,segments,*_=sim.prepare(rows,150)
        self.assertEqual(segments,[[0,1,2],[3,4]])
        replay=sim.resample(t,data,segments,"angle_only")
        self.assertFalse(any(210000 < r['time_us'] < 610000 for r in replay))
        self.assertEqual([r['left_v'] for r in replay if r['time_us'] in [10000,610000]],[0,0])
        self.assertTrue(all(abs(r['left_v']-100)<1e-8 for r in replay if r['time_us'] not in [10000,610000]))

    def test_missing_rows_do_not_become_zero(self):
        rows=[row(t) for t in [0,100,200,300,400]]
        rows[2]['leftPositionDeg']=None
        self.assertEqual(sim.prepare(rows,150)[2],[[0,1],[3,4]])
        self.assertTrue(np.isnan(sim.number(None)))
        self.assertTrue(np.isnan(sim.number('')))

    def test_invalid_order_and_changing_config_rejected(self):
        for rows in ([row(0),row(0)],[row(100),row(0)]):
            with self.assertRaises(ValueError): sim.prepare(rows,150)
        rows=[row(0),row(100)]; rows[1]['maximumNm']=5
        with self.assertRaises(ValueError): sim.prepare(rows,150)
        rows=[row(0),row(100)]; rows[1]['outputAllowed']=False
        with self.assertRaises(ValueError): sim.prepare(rows,150)

    def test_context_does_not_backfill_future_event(self):
        context={'info':{'firmware':[0,11,21]},'assistConfig':{'strategyName':'ADAPTIVE_GAUSSIAN','leftLevelPercent':100,'rightLevelPercent':100},'assistTorqueLimit':{'selectedMaximumNm':15}}
        later={**context,'assistTorqueLimit':{'selectedMaximumNm':5}}
        objects=[dict(type='session',startedAt='2026-11-18T00:00:00Z',context=context,
            events=[dict(type='device_state',hostTime='2026-11-18T00:00:02Z',value=later)]),
            dict(type='sample',receivedIsoTime='2026-11-18T00:00:01Z'),
            dict(type='sample',receivedIsoTime='2026-11-18T00:00:03Z')]
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'test.jsonl'
            p.write_text('\n'.join(json.dumps(o) for o in objects),encoding='utf-8')
            rows,_=sim.read_recording(p)
        self.assertEqual([r['maximumNm'] for r in rows],[15,5])


if __name__=='__main__':
    unittest.main()
