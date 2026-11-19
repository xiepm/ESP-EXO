"""Offline tests of acceptance/rejection logic; no serial import or device access."""
import unittest
from nominal_peak_run import verify_peak, verify_idle, verify_closed_idle, PLAN, validate_fixture
from argparse import Namespace

GOOD = "BENCHPEAK profile=2 initialized=1 complete=1 fault=0 side=0 requested=15 generated_peak=15 sent_peak=15 estimate_peak=15.1 plateau_mean=14.9 plateau_samples=8 samples=301 max_displacement=0.1 max_speed=0.2 max_temperature=30 start_us=1000000 last_us=4000000"


class AcceptanceTests(unittest.TestCase):
    def test_fixture_is_output_constraint_not_waist_fixation(self):
        for value in (None, 'free-leg'):
            with self.assertRaises(RuntimeError):
                validate_fixture(Namespace(fixture_type=value,fixture_confirmed=True))
        validate_fixture(Namespace(fixture_type='restrained-output'))

    def test_valid_result(self):
        self.assertEqual(verify_peak(GOOD, "left", 15, 0)["plateau_samples"], 8)

    def test_reject_invalid_results(self):
        for before, after in (("complete=1", "complete=0"), ("fault=0", "fault=3"),
                              ("side=0", "side=1"), ("requested=15 ", "requested=-15 "),
                              ("sent_peak=15 ", "sent_peak=5 "), ("estimate_peak=15.1", "estimate_peak=5"),
                              ("plateau_mean=14.9", "plateau_mean=nan"),
                              ("plateau_mean=14.9", "plateau_mean=10"),
                              ("plateau_samples=8", "plateau_samples=5"),
                              ("max_displacement=0.1", "max_displacement=2.1"),
                              ("profile=2", "profile=1"),
                              ("last_us=4000000", "last_us=3100000"),
                              ("last_us=4000000", "last_us=4100000")):
            with self.subTest(after=after), self.assertRaises(RuntimeError):
                verify_peak(GOOD.replace(before, after), "left", 15, 0)
        with self.assertRaises(RuntimeError):
            verify_peak(GOOD, "left", 15, 1000000)
        with self.assertRaises(RuntimeError):
            verify_peak(GOOD + "\n" + GOOD, "left", 15, 0)

    def test_idle_contract(self):
        good = "State: IDLE\nRun requested: no\nOutput allowed: no\nRuntime faults: 0; last=NONE"
        verify_idle(good)
        for bad in (good.replace("IDLE", "RUNNING"), good.replace("0;", "1;"), good.replace("allowed: no", "allowed: YES")):
            with self.assertRaises(RuntimeError):
                verify_idle(bad)

    def test_plan_bounded(self):
        self.assertEqual(len(PLAN["sequence"]), 20)
        self.assertEqual({s["side"] for s in PLAN["sequence"]}, {"left", "right"})
        self.assertEqual(max(abs(s["signed_nm"]) for s in PLAN["sequence"]), 15)

    def test_cleanup_checks_idle_despite_recorded_fault(self):
        text = "State: IDLE\nRun requested: no\nOutput allowed: no\nRuntime faults: 1; last=CONTROL_OUTPUT_FAILED"
        verify_closed_idle(text)
        with self.assertRaises(RuntimeError):
            verify_idle(text)
        with self.assertRaises(RuntimeError):
            verify_closed_idle(text.replace("IDLE", "ERROR"))


if __name__ == "__main__":
    unittest.main()
