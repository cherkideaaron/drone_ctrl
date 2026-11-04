import argparse, math, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_lib import CController, MODE_NAMES, LANDED
from physics import QuadPhysics

DT = 0.005


def run_headless(hover_height=2.0, climb_speed=1.0, hover_time=5.0, descend_speed=0.7,
                 diag_horiz=2.0, diag_angle=45.0, diag_speed=1.0, far_hold=1.5,
                 mass=1.5, thrust_to_weight=2.0, t_max=180.0, verbose=True,
                 kick=None, phys_kw=None):
    ctrl = CController(hover_height, climb_speed, hover_time, descend_speed,
                       diag_horiz, diag_angle, diag_speed, far_hold)
    phys = QuadPhysics(mass=mass, thrust_to_weight=thrust_to_weight, **(phys_kw or {}))
    ctrl.start()

    t, last_mode = 0.0, 0
    res = {"events": [], "trace": [], "max_tilt_deg": 0.0, "kick_t": None}
    kicked = False
    hover_start = None
    n = 0
    while t < t_max:
        mode, motors, st = ctrl.step(phys.sensors(), DT)
        phys.step(motors, DT)
        t += DT
        n += 1
        res["max_tilt_deg"] = max(res["max_tilt_deg"], phys.tilt_deg())
        if n % 4 == 0:
            res["trace"].append((t, mode, phys.x, phys.y, phys.z, phys.roll, phys.pitch,
                                 st["sp"][0], st["sp"][1], st["sp"][2]))
        if mode != last_mode:
            res["events"].append((t, MODE_NAMES[mode], phys.x, phys.y, phys.z))
            if verbose:
                print(f"  t={t:6.2f}s  -> {MODE_NAMES[mode]:9s} x={phys.x:5.2f}  height={phys.z:5.2f} m")
            if MODE_NAMES[mode] == "HOVER" and hover_start is None:
                hover_start = t
            last_mode = mode
        if kick and not kicked and hover_start is not None and t - hover_start >= kick[0]:
            phys.kick(kick[1], kick[2])
            res["kick_t"] = t
            kicked = True
        if mode == LANDED:
            for _ in range(int(1.0 / DT)):
                m2, motors, st = ctrl.step(phys.sensors(), DT)
                phys.step(motors, DT)
            break

    res["landed"] = last_mode == LANDED
    res["time"] = t
    res["impact_speed"] = phys.impact_speed
    res["final_pos"] = (phys.x, phys.y, phys.z)
    return res


def run_view(args):
    from quad_view import QuadView
    cfg = dict(hover_height=args.hover_height, climb_speed=args.climb_speed,
               hover_time=args.hover_time, descend_speed=args.descend_speed,
               diag_horiz=args.diag_horiz, diag_angle=args.diag_angle,
               diag_speed=args.diag_speed, far_hold=args.far_hold)
    ctrl = CController(**cfg)
    phys = QuadPhysics()
    top = args.hover_height + args.diag_horiz * math.tan(math.radians(args.diag_angle))
    view = QuadView(args.diag_horiz, top)
    steps_per_frame = 3
    fps = 1.0 / (DT * steps_per_frame) * args.speed
    last_mode = 0
    if args.autostart:
        view.start_requested = True

    while True:
        view.wait_frame(fps)
        if view.reset_requested:
            view.reset_requested = False
            phys.reset()
            ctrl.configure(**cfg)
            last_mode = 0
            view.clear_trail()
        if view.start_requested:
            view.start_requested = False
            ctrl.start()
            print("Start requested")
        if view.gust_requested:
            view.gust_requested = False
            if phys.z > 0.3:
                phys.kick(roll_rate=3.0, pitch_rate=2.0)
                print("Gust!")

        for _ in range(steps_per_frame):
            mode, motors, st = ctrl.step(phys.sensors(), DT)
            phys.step(motors, DT)

        if mode != last_mode:
            print(f"  -> {MODE_NAMES[mode]:9s} x={phys.x:5.2f}  height={phys.z:5.2f} m")
            last_mode = mode
        view.update(phys, phys.motor_actual, st["sp"], MODE_NAMES[mode], mode, st["thrust"], st["mode_time"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--hover-height", type=float, default=2.0, help="m")
    ap.add_argument("--climb-speed", type=float, default=1.0, help="m/s")
    ap.add_argument("--hover-time", type=float, default=5.0, help="s")
    ap.add_argument("--descend-speed", type=float, default=0.7, help="m/s")
    ap.add_argument("--diag-horiz", type=float, default=2.0, help="horizontal length of the 45-deg leg, m (0 = skip)")
    ap.add_argument("--diag-angle", type=float, default=45.0, help="deg above horizontal")
    ap.add_argument("--diag-speed", type=float, default=1.0, help="m/s along the diagonal")
    ap.add_argument("--far-hold", type=float, default=1.5, help="pause at the far end, s")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (0.5 = slow motion)")
    ap.add_argument("--autostart", action="store_true")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    if args.headless:
        r = run_headless(args.hover_height, args.climb_speed, args.hover_time, args.descend_speed,
                         args.diag_horiz, args.diag_angle, args.diag_speed, args.far_hold)
        x, y, z = r["final_pos"]
        print(f"\nlanded={r['landed']}  flight time={r['time']:.1f}s  max tilt={r['max_tilt_deg']:.1f} deg  "
              f"landing offset from start=({x*100:.0f}, {y*100:.0f}) cm  touchdown speed={r['impact_speed']:.2f} m/s")
    else:
        run_view(args)
