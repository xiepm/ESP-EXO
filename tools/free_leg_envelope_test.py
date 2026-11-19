import unittest
from free_leg_envelope import simulate, SCENARIOS


class ModelTests(unittest.TestCase):
    def test_rest_stays_at_rest(self):
        r=simulate(.009,.15,0,.01)
        self.assertEqual(r['peak_abs_angle_deg'],0)
        self.assertEqual(r['peak_abs_speed_deg_s'],0)

    def test_sign_symmetry(self):
        a=simulate(.009,.15,.1,.01)
        b=simulate(.009,.15,-.1,.01)
        self.assertAlmostEqual(a['peak_abs_angle_deg'],b['peak_abs_angle_deg'],places=9)

    def test_step_convergence_and_energy(self):
        for inertia,center in SCENARIOS.values():
            a=simulate(inertia,center,.25,.01)
            b=simulate(inertia,center,.25,.01,dt=.00025)
            self.assertLess(abs(a['peak_abs_angle_deg']-b['peak_abs_angle_deg']),.001)
            self.assertLess(a['max_energy_balance_error_j'],1e-7)

    def test_nominal_max_not_low_energy(self):
        for inertia,center in SCENARIOS.values():
            self.assertFalse(simulate(inertia,center,15,.01)['passes_proposed_model_screen'])


if __name__=='__main__':unittest.main()
