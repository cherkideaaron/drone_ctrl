import argparse, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_lib import CController, MODE_NAMES
from physics import QuadPhysics

DT = 0.005


def run_headless(hover_height=2.0, climb_speed=1.0, hover_time=5.0, descend_speed=0.7,
                 mass=1.5, thrust_to_weight=2.0, t_max=120.0, verbose=True):
    ctrl = CController(hover_height, climb_speed, hover_time, descend_speed)
    phys = QuadPhysics(mass=mass, thrust_to_weight=thrust_to_weight)
    ctrl.start()

    t, last_mode = 0.0, 0
    log = {"max_height": 0.0, "events": [], "hover_err_max": 0.0, "hover_samples": 0}
    hover_start_t = None
    while t < t_max:
        h, vz = phys.sensors()
        mode, motors, st = ctrl.step(h, vz, DT)
        phys.step(motors, DT)
        t += DT
        log["max_height"] = max(log["max_height"], phys.height)

        if mode != last_mode:
            log["events"].append((t, MODE_NAMES[mode], phys.height))
            if verbose:
                print(f"  t={t:6.2f}s  -> {MODE_NAMES[mode]:8s} height={phys.height:5.2f} m")
            if MODE_NAMES[mode] == "HOVER":
                hover_start_t = t
            last_mode = mode

        if MODE_NAMES[mode] == "HOVER" and t - hover_start_t > 1.0:
            log["hover_err_max"] = max(log["hover_err_max"], abs(phys.height - hover_height))
            log["hover_samples"] += 1

        if MODE_NAMES[mode] == "LANDED":
            for _ in range(int(1.0 / DT)):
                h, vz = phys.sensors()
                _, motors, _ = ctrl.step(h, vz, DT)
                phys.step(motors, DT)
            break

    log["landed"] = MODE_NAMES[last_mode] == "LANDED"
    log["time"] = t
    log["impact_speed"] = phys.impact_speed
    return log


def run_view(args):
    from quad_view import QuadView
    ctrl = CController(args.hover_height, args.climb_speed, args.hover_time, args.descend_speed)
    phys = QuadPhysics()
    view = QuadView(args.hover_height)
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
            ctrl.configure(args.hover_height, args.climb_speed, args.hover_time, args.descend_speed)
            last_mode = 0
            view.set_mode_text("IDLE")
        if view.start_requested:
            view.start_requested = False
            ctrl.start()

        for _ in range(steps_per_frame):
            h, vz = phys.sensors()
            mode, motors, st = ctrl.step(h, vz, DT)
            phys.step(motors, DT)

        if mode != last_mode:
            print(f"  -> {MODE_NAMES[mode]:8s} height={phys.height:5.2f} m")
            last_mode = mode
        view.update(phys.height, phys.vz, phys.motor_actual, st["sp_height"],
                    MODE_NAMES[mode], st["thrust"], st["mode_time"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--hover-height",   type=float, default=2.0)
    ap.add_argument("--climb-speed",    type=float, default=1.0)
    ap.add_argument("--hover-time",     type=float, default=5.0)
    ap.add_argument("--descend-speed",  type=float, default=0.7)
    ap.add_argument("--speed",          type=float, default=1.0)
    ap.add_argument("--autostart",      action="store_true")
    ap.add_argument("--headless",       action="store_true")
    args = ap.parse_args()

    if args.headless:
        r = run_headless(args.hover_height, args.climb_speed, args.hover_time, args.descend_speed)
        print(f"\nlanded={r['landed']}  flight time={r['time']:.1f}s  max height={r['max_height']:.2f} m  "
              f"hover error={r['hover_err_max']*100:.1f} cm  touchdown speed={r['impact_speed']} m/s")
    else:
        run_view(args)