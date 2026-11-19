"""Offline free-leg screening. No serial/BLE, parameter writes or motion commands.

User inputs: each side 0.3kg, 0.3m, initially vertical down. Mass distribution,
motor/reflected inertia, friction and true delivered torque are NOT measured.
Scenario predictions are not guaranteed physical bounds or deployment approval.
"""
import argparse
import json
import math
from pathlib import Path

MASS = .3
LENGTH = .3
G = 9.80665
SCENARIOS = {
    'mass_at_quarter_length': (MASS*(LENGTH/4)**2, LENGTH/4),
    'uniform_rod': (MASS*LENGTH**2/3, LENGTH/2),
    'mass_at_tip': (MASS*LENGTH**2, LENGTH),
}


def simulate(inertia, center, torque, hold_s, dt=.0005, duration=4):
    """Ideal joint torque square pulse followed by zero; no friction or damping.

    This compares impulses, not the existing fixed-output profile. Integration
    steps end exactly at the switch; each phase uses its own held torque.
    """
    if inertia <= 0 or not 0 < center <= LENGTH or hold_s <= 0 or dt <= 0:
        raise ValueError('invalid model')
    q = v = t = work = 0.0
    max_q = max_v = max_energy_error = 0.0
    def accel(angle, command):
        return (command-MASS*G*center*math.sin(angle))/inertia
    while t < duration-1e-12:
        command = torque if t < hold_s-1e-12 else 0.0
        step = min(dt, duration-t)
        if t < hold_s-1e-12:
            step = min(step, hold_s-t)
        a1 = accel(q, command)
        q2 = q + step*v/2; v2 = v + step*a1/2
        a2 = accel(q2, command)
        q3 = q + step*v2/2; v3 = v + step*a2/2
        a3 = accel(q3, command)
        q4 = q + step*v3; v4 = v + step*a3
        nq = q + step*(v+2*v2+2*v3+v4)/6
        nv = v + step*(a1+2*a2+2*a3+accel(q4, command))/6
        work += command*(nq-q)
        q,v,t = nq,nv,t+step
        energy = .5*inertia*v*v + MASS*G*center*(1-math.cos(q))
        max_energy_error = max(max_energy_error,abs(energy-work))
        max_q = max(max_q,abs(math.degrees(q)))
        max_v = max(max_v,abs(math.degrees(v)))
    return {'peak_abs_angle_deg':max_q,'peak_abs_speed_deg_s':max_v,
            'max_energy_balance_error_j':max_energy_error,
            'passes_proposed_model_screen':max_q<=5 and max_v<=20}


def report():
    cases=[]
    for name,(inertia,center) in SCENARIOS.items():
        for torque,hold in ((.02,.01),(.05,.01),(.1,.01),(.25,.01),(1,.01),(15,.01)):
            cases.append({'scenario':name,'inertia_kg_m2':inertia,'center_m':center,
                          'torque_nm':torque,'hold_s':hold,
                          **simulate(inertia,center,torque,hold)})
    return {'source':'operator-reported geometry/mass; scenario mass distributions assumed',
            'mass_kg_each':MASS,'length_m_each':LENGTH,'initial':'vertical down, zero speed',
            'gravity_torque_upper_nm_if_all_mass_within_length':MASS*G*LENGTH,
            'hardware_verified':False,'deployment_allowed_by_this_report':False,
            'proposed_screen_only':{'angle_deg':5,'speed_deg_s':20},
            'limitations':['No proven lower inertia bound from total mass/length alone.',
                'No rotor inertia, friction, gearbox dynamics or torque-estimator error model.',
                'A 10ms ideal pulse is not proof of MCU timing or actual actuator response.',
                'No damping means motion persists after command becomes zero.',
                'Physical excursion allowance has not been measured.'], 'cases':cases}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    with args.output.open('x',encoding='utf-8') as f:json.dump(report(),f,indent=2)
    print(args.output)
