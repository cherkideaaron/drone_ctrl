import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_lib import CController, mixer, MODE_NAMES, OUTBOUND, RETURN, DESCEND
from run_sim import run_headless, DT

results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}  {detail}")


def event(r, mode):
    for e in r["events"]:
        if e[1] == mode:
            return e
    return None


def dist_to_line(p, a, b):
    v = [b[i] - a[i] for i in range(3)]
    w = [p[i] - a[i] for i in range(3)]
    t = max(0.0, min(1.0, sum(w[i] * v[i] for i in range(3)) / sum(c * c for c in v)))
    return math.dist(p, [a[i] + t * v[i] for i in range(3)])


def line_dev(r, mode, a, b):
    pts = [(x, y, z) for (t, m, x, y, z, *_) in r["trace"] if m == mode]
    return max(dist_to_line(p, a, b) for p in pts)


print("\n1) mixer_update() - layer 8: torque goes to the right motors, output always 0..1")
m = mixer(0.5)
check("zero torque -> all 4 motors equal", max(m) - min(m) < 1e-6, f"{m}")
m = mixer(0.5, pitch=0.1)
check("pitch up -> front motors (0,2) faster than rear (1,3)", m[0] > m[1] and m[2] > m[3])
m = mixer(0.5, roll=0.1)
check("roll right -> left motors (1,2) faster than right (0,3)", m[1] > m[0] and m[2] > m[3])
m = mixer(0.98, roll=0.5)
check("motor outputs clamped to 0..1", all(0.0 <= x <= 1.0 for x in m))


print("\n2) mode_update() - layer 3: nothing happens until Start; phases run in the right order")
c = CController()
mode, motors, _ = c.step([0.0] * 10, DT)
check("IDLE before start, motors off", MODE_NAMES[mode] == "IDLE" and max(motors) == 0.0)
r = run_headless(verbose=False)
order = [e[1] for e in r["events"]]
check("order: LIFT > HOVER > OUTBOUND > FAR_HOLD > RETURN > DESCEND > LANDED",
      order == ["LIFT", "HOVER", "OUTBOUND", "FAR_HOLD", "RETURN", "DESCEND", "LANDED"], str(order))
rv = run_headless(diag_horiz=0.0, verbose=False)
check("diag_horiz = 0 skips the diagonal (lift / hover / descend only)",
      [e[1] for e in rv["events"]] == ["LIFT", "HOVER", "DESCEND", "LANDED"])


print("\n3) setpoint_update() + altitude_control() - vertical flight (diagonal switched off)")
hov = event(rv, "HOVER"), event(rv, "DESCEND")
hover_len = hov[1][0] - hov[0][0]
zs = [z for (t, m, x, y, z, *_) in rv["trace"] if hov[0][0] + 1.0 < t < hov[1][0]]
check("hover holds 2.0 m within 3 cm", max(abs(z - 2.0) for z in zs) < 0.03,
      f"{max(abs(z - 2.0) for z in zs) * 100:.1f} cm")
check("overshoot while climbing below 10 cm", max(z for (t, m, x, y, z, *_) in rv["trace"]) - 2.0 < 0.10)
check("hover lasts the requested 5 s (+-0.1)", abs(hover_len - 5.0) < 0.1, f"{hover_len:.2f} s")
check("soft touchdown below 0.5 m/s", rv["impact_speed"] < 0.5, f"{rv['impact_speed']:.2f} m/s")


print("\n4) attitude_control() + rate_control() - layers 6 and 7: balance after a sudden gust in hover")
cases = [("nominal drone", dict()),
         ("thrust/weight 1.6", dict(thrust_to_weight=1.6)),
         ("slow motors (70 ms lag)", dict(phys_kw=dict(motor_tau=0.07))),
         ("heavier rotation inertia (+33 %)", dict(phys_kw=dict(inertia=0.04))),
         ("lighter rotation inertia (-33 %)", dict(phys_kw=dict(inertia=0.02, motor_tau=0.03)))]
for name, kw in cases:
    rk = run_headless(kick=(1.5, 3.0, 3.0), verbose=False, **kw)
    kt = rk["kick_t"]
    peak = max(math.degrees(max(abs(ro), abs(pi))) for (t, m, x, y, z, ro, pi, *_) in rk["trace"] if kt <= t <= kt + 3)
    late = max(math.degrees(max(abs(ro), abs(pi))) for (t, m, x, y, z, ro, pi, *_) in rk["trace"] if kt + 1.5 <= t <= kt + 3)
    drift = max(math.dist((x, y, z), (0, 0, 2.0)) for (t, m, x, y, z, *_) in rk["trace"] if kt <= t <= kt + 3)
    check(f"{name}: level again 1.5 s after the kick, drift < 30 cm, mission completes",
          late < 3.0 and drift < 0.30 and peak < 30.0 and rk["landed"] and
          math.hypot(*rk["final_pos"][:2]) < 0.10,
          f"peak {peak:.1f} deg, tilt after 1.5 s {late:.2f} deg, drift {drift*100:.0f} cm")


print("\n5) mission - the 4-step sequence: lift+hover, 45 deg out, same route back, vertical descent")
hover_pt, far_pt = (0.0, 0.0, 2.0), (2.0, 0.0, 4.0)
out_end, ret_start, ret_end = event(r, "FAR_HOLD"), event(r, "RETURN"), event(r, "DESCEND")
check("step 1: hover point reached (2.0 m up, above the start)",
      math.dist(event(r, "HOVER")[2:], hover_pt) < 0.06, f"{[round(v, 2) for v in event(r, 'HOVER')[2:]]}")
check("step 2: far end reached (2 m forward, 2 m up)", math.dist(out_end[2:], far_pt) < 0.08,
      f"{[round(v, 2) for v in out_end[2:]]}")
ang = math.degrees(math.atan2(out_end[4] - event(r, "OUTBOUND")[4], out_end[2] - event(r, "OUTBOUND")[2]))
check("step 2: flown at 45 deg above horizontal (+-2)", abs(ang - 45.0) < 2.0, f"{ang:.1f} deg")
d_out, d_back = line_dev(r, OUTBOUND, hover_pt, far_pt), line_dev(r, RETURN, hover_pt, far_pt)
check("step 2: stays within 6 cm of the straight line", d_out < 0.06, f"{d_out*100:.1f} cm")
check("step 3: return follows the SAME line (within 6 cm)", d_back < 0.06, f"{d_back*100:.1f} cm")
check("step 3: back at the hover point (within 6 cm)", math.dist(ret_end[2:], hover_pt) < 0.06,
      f"{[round(v, 2) for v in ret_end[2:]]}")
xy = [math.hypot(x, y) for (t, m, x, y, z, *_) in r["trace"] if m == DESCEND and z > 0.3]
check("step 4: descent is vertical (horizontal drift < 5 cm)", max(xy) < 0.05, f"{max(xy)*100:.1f} cm")
check("step 4: lands on the starting point (within 10 cm)", math.hypot(*r["final_pos"][:2]) < 0.10,
      f"{math.hypot(*r['final_pos'][:2])*100:.1f} cm")
check("soft touchdown below 0.5 m/s", r["impact_speed"] < 0.5, f"{r['impact_speed']:.2f} m/s")
check("tilt stays below 15 deg", r["max_tilt_deg"] < 15.0, f"{r['max_tilt_deg']:.1f} deg")
hl = event(r, "OUTBOUND")[0] - event(r, "HOVER")[0]
check("hover lasts the requested 5 s (+-0.1)", abs(hl - 5.0) < 0.1, f"{hl:.2f} s")


print("\n6) adjustable distance, angle, speed and hover time")
for kw, far in [(dict(diag_horiz=1.0, diag_speed=0.5, hover_time=3.0), (1.0, 0.0, 3.0)),
                (dict(diag_horiz=3.0, diag_speed=1.5, hover_time=8.0, hover_height=1.5), (3.0, 0.0, 4.5)),
                (dict(diag_horiz=2.0, diag_angle=30.0), (2.0, 0.0, 2.0 + 2.0 * math.tan(math.radians(30))))]:
    rr = run_headless(verbose=False, **kw)
    fe, hv, ob = event(rr, "FAR_HOLD"), event(rr, "HOVER"), event(rr, "OUTBOUND")
    hl2 = ob[0] - hv[0]
    hh = kw.get("hover_height", 2.0)
    d = line_dev(rr, OUTBOUND, (0, 0, hh), far)
    check(f"{kw}", rr["landed"] and math.dist(fe[2:], far) < 0.08 and d < 0.08 and
          abs(hl2 - kw.get("hover_time", 5.0)) < 0.1 and math.hypot(*rr["final_pos"][:2]) < 0.10,
          f"far end {[round(v, 2) for v in fe[2:]]}, line dev {d*100:.1f} cm, hover {hl2:.1f} s")

print("\n%d / %d checks passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
