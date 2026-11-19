import unittest
from stationary_window_candidate import evaluate


class WindowTests(unittest.TestCase):
    def setUp(self):
        self.rows = [(i*.1, .022*(i%2), (-1)**i*6, 0, True) for i in range(31)]
        self.rows = [(t,q,v if i%3 == 0 else 0,tau,ok)
                     for i,(t,q,v,tau,ok) in enumerate(self.rows)]

    def test_zero_mean_fluctuation(self):
        self.assertTrue(evaluate(self.rows)[0])

    def test_drift_and_oscillation(self):
        for angles in ([i*.01 for i in range(31)], [(-1)**i*.15 for i in range(31)]):
            self.assertFalse(evaluate([(r[0],q,*r[2:]) for r,q in zip(self.rows,angles)])[0])

    def test_persistent_velocity(self):
        self.assertFalse(evaluate([(t,q,6,tau,ok) for t,q,v,tau,ok in self.rows])[0])

    def test_bad_samples(self):
        for index,value in ((0,-1),(1,float('nan')),(2,21),(3,.6),(4,False)):
            rows=list(self.rows); r=list(rows[15]); r[index]=value; rows[15]=tuple(r)
            self.assertFalse(evaluate(rows)[0])

    def test_missing_coverage(self):
        self.assertFalse(evaluate(self.rows[:10])[0])
        self.assertFalse(evaluate(self.rows[:10]+self.rows[15:])[0])


if __name__ == '__main__':
    unittest.main()
