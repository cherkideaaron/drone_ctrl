import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_lib import CController, mixer, MODE_NAMES
from run_sim import run_headless, DT

results = []


def check(name, cond, detail=""):
    results.append(cond)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}  {detail}")


def event_time(r, mode):
    for t, m, _ in r["events"]:
        if m == mode:
            return t
    return None


print("\n1) mixer_update()  - layer 8")
m = mixer(0.5)
check("zero torque -> all 4 motors equal", max(m) - min(m) < 1e-6, f"{m}")
m = mixer(0.5, pitch=0.1)
check("pitch torque -> front motors (0,2) faster than rear (1,3)",
      m[0] > m[1] and m[2] > m[3], f"{[round(x, 2) for x in m]}")
m = mixer(0.98, roll=0.5)
check("motor outputs always clamped to 0..1", all(0.0 <= x <= 1.0 for x in m), f"{m}")

print("\n2) mode_update()  - layer 3")
c = CController()
mode, motors, _ = c.step(0.0, 0.0, DT)
check("IDLE before start, motors off", MODE_NAMES[mode] == "IDLE" and max(motors) == 0.0)
r = run_headless(verbose=False)
order = [e[1] for e in r["events"]]
check("phase order LIFT -> HOVER -> DESCEND -> LANDED", order == ["LIFT", "HOVER", "DESCEND", "LANDED"], str(order))

print("\n3) setpoint_update() + altitude_control()  - layers 4 and 5")
check("reaches hover height", r["events"][1][2] > 1.9, f"{r['events'][1][2]:.2f} m at start of hover")
check("hover holds height within 3 cm", r["hover_err_max"] < 0.03, f"{r['hover_err_max']*100:.1f} cm")
check("overshoot below 10 cm", r["max_height"] - 2.0 < 0.10, f"{(r['max_height']-2.0)*100:.1f} cm")
hover_len = event_time(r, "DESCEND") - event_time(r, "HOVER")
check("hover lasts the requested 5 s (+-0.1)", abs(hover_len - 5.0) < 0.1, f"{hover_len:.2f} s")
check("soft touchdown below 0.5 m/s", r["impact_speed"] is not None and r["impact_speed"] < 0.5,
      f"{r['impact_speed']:.2f} m/s")

print("\n4) adjustable parameters")
for hh, cs, ht, ds in [(1.0, 0.5, 2.0, 0.5), (3.0, 1.5, 8.0, 1.0)]:
    r2 = run_headless(hh, cs, ht, ds, verbose=False)
    hl = event_time(r2, "DESCEND") - event_time(r2, "HOVER")
    check(f"height={hh} m speed={cs} hover={ht} s",
          r2["landed"] and r2["hover_err_max"] < 0.03 and abs(hl - ht) < 0.1 and
          r2["max_height"] - hh < 0.12 * hh,
          f"hover err {r2['hover_err_max']*100:.1f} cm, hover {hl:.1f} s, max {r2['max_height']:.2f} m")

print("\n5) robustness: real drone differs from what controller assumes")
for tw in (1.6, 2.5, 3.0):
    r3 = run_headless(thrust_to_weight=tw, verbose=False)
    check(f"thrust/weight = {tw} (true hover thrust {1/tw:.2f})",
          r3["landed"] and r3["hover_err_max"] < 0.05 and r3["impact_speed"] < 0.6,
          f"hover err {r3['hover_err_max']*100:.1f} cm, touchdown {r3['impact_speed']:.2f} m/s")

print("\n%d / %d checks passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)