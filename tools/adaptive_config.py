"""Generate a reviewable UART command file and versioned CRC blob. Never opens a port."""
import argparse,json,math,struct,zlib
from pathlib import Path
AFO="minimum_hz maximum_hz initial_hz phase_gain frequency_gain amplitude_gain input_scale_deg maximum_dt_s velocity_filter_tau_s event_velocity_deg_s event_excursion_deg correction_rate_rad_s".split()
LEG="enabled strength flex_amplitude_nm flex_peak_phase flex_width ext_amplitude_nm ext_peak_phase ext_width torque_limit_nm slew_nm_s positive_power_w negative_power_w minimum_angle_deg maximum_angle_deg maximum_velocity_deg_s fade_s transition_s".split()
def encode(c):
    if set(c)!={"schema","overall_strength","afo","left","right"} or c["schema"]!=1:raise ValueError("schema/fields")
    for group,keys in [("afo",AFO),("left",LEG),("right",LEG)]:
        if set(c[group])!=set(keys):raise ValueError(group+" fields")
    values=[c["overall_strength"]]+[c["afo"][k] for k in AFO]+[c[side][k] for side in ["left","right"] for k in LEG]
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in values):raise ValueError("nonfinite/non-numeric parameter")
    if c["afo"]["maximum_dt_s"]>.03:raise ValueError("target control loop maximum_dt_s must be <=0.03")
    data=struct.pack("<I47f",1,*values)
    return data+struct.pack("<I",zlib.crc32(data))
def commands(c):
    lines=["# Generated text only; review and C-validate blob before using on the IDLE bench.",
           "# No START/enable/zero command. New strategy ACTIVE is blocked in stages1-4.",
           "adaptive_store defaults"]
    for k in AFO:lines.append(f"adaptive_set afo {k} {c['afo'][k]:.8g}")
    for side in ["left","right"]:
        for k in LEG:
            if k!="enabled":lines.append(f"adaptive_set {side} {k} {c[side][k]:.8g}")
        lines.append(f"adaptive_set {side} enabled {c[side]['enabled']:.8g}")
    lines.extend([f"adaptive_set global overall_strength {c['overall_strength']:.8g}","adaptive_config"])
    return "\n".join(lines)+"\n"
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("config",type=Path);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();c=json.loads(a.config.read_text(encoding="utf-8"))
    b=encode(c);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.with_suffix(".bin").write_bytes(b)
    a.output.with_suffix(".txt").write_text(commands(c),encoding="utf-8")
    print(f"Generated {len(b)} bytes and commands. Run adaptive_replay --validate-config {a.output.with_suffix('.bin')} for actual firmware range validation.")
